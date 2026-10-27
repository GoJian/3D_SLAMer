"""rosbag2 helpers (MCAP by default).

`record_time_ns` is the time the recorder received a message. It is used ONLY for latency
statistics, never as acquisition time; acquisition time is header.stamp.
"""
from __future__ import annotations

import struct
from pathlib import Path
from typing import Iterator, NamedTuple

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

# Message types whose first field is std_msgs/Header; their stamp can be read without full
# deserialization (fast for large images).
HEADER_FIRST_TYPES = {
    'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CameraInfo',
    'sensor_msgs/msg/Imu', 'nav_msgs/msg/Odometry', 'geometry_msgs/msg/PoseStamped',
    'x650_msgs/msg/FrameMetadata', 'diagnostic_msgs/msg/DiagnosticArray',
}


class BagMessage(NamedTuple):
    topic: str
    type: str
    data: bytes
    record_time_ns: int


def header_stamp_ns(data: bytes) -> int:
    """Read header.stamp from a CDR-serialized message that starts with std_msgs/Header."""
    # 4-byte encapsulation header, then int32 sec, uint32 nanosec.
    little = data[1] == 0x01
    sec, nsec = struct.unpack('<iI' if little else '>iI', data[4:12])
    return sec * 1_000_000_000 + nsec


def _storage_id(path: Path) -> str:
    p = Path(path)
    if p.is_dir():
        if any(p.glob('*.db3')):
            return 'sqlite3'
    elif p.suffix == '.db3':
        return 'sqlite3'
    return 'mcap'


def open_reader(path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=_storage_id(Path(path))),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def topic_types(path) -> dict[str, str]:
    reader = open_reader(path)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_messages(path, topics=None) -> Iterator[BagMessage]:
    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield BagMessage(topic, types[topic], data, t)


def deserialize(msg: BagMessage):
    return deserialize_message(msg.data, get_message(msg.type))


class BagWriter:
    """Minimal MCAP writer for synthetic sessions and tests."""

    def __init__(self, path, storage_id: str = 'mcap'):
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
        self._topics: set[str] = set()

    def write(self, topic: str, msg, record_time_ns: int) -> None:
        if topic not in self._topics:
            mod = type(msg).__module__.split('.')[0]
            type_name = f'{mod}/msg/{type(msg).__name__}'
            self._writer.create_topic(rosbag2_py.TopicMetadata(
                id=len(self._topics), name=topic, type=type_name, serialization_format='cdr'))
            self._topics.add(topic)
        self._writer.write(topic, serialize_message(msg), int(record_time_ns))

    def close(self) -> None:
        del self._writer
