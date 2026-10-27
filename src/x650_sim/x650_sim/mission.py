"""Offboard test mission for PX4 SITL with vision-only navigation.

Waits for a valid local position (from external vision), switches to offboard, arms, takes off,
flies a square, optionally injects faults into the fake VIO at chosen waypoints, then lands.
Setpoints are in PX4's local frame (here the FRD frame defined by external vision).

  ros2 run x650_sim x650_sim_mission --ros-args -p altitude_m:=2.5 -p side_m:=4.0

Exit code 0 when it lands and disarms, 1 on timeout or failsafe.
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field


@dataclass
class Waypoint:
    x: float
    y: float
    z: float                 # PX4 local, down positive
    yaw: float
    fault: str | None = None  # injected into the fake VIO when this waypoint is reached


@dataclass
class MissionPlan:
    waypoints: list[Waypoint] = field(default_factory=list)


def square_plan(altitude_m=2.5, side_m=4.0, faults: dict[int, str] | None = None) -> MissionPlan:
    """Takeoff above the origin, fly a square (yaw follows the leg), return, hold."""
    faults = faults or {}
    z = -abs(altitude_m)
    s = side_m
    pts = [(0, 0, 0.0), (s, 0, 0.0), (s, s, math.pi / 2), (0, s, math.pi), (0, 0, -math.pi / 2),
           (0, 0, 0.0)]
    wps = [Waypoint(x, y, z, yaw, faults.get(i)) for i, (x, y, yaw) in enumerate(pts)]
    return MissionPlan(wps)


def reached(pos, wp: Waypoint, tol_m: float) -> bool:
    return math.dist(pos, (wp.x, wp.y, wp.z)) < tol_m


def parse_faults(spec: str) -> dict[int, str]:
    """'2:jump 0.5;3:dropout 1.0;4:reset' -> {2: 'jump 0.5', ...}"""
    out = {}
    for item in filter(None, (s.strip() for s in spec.split(';'))):
        idx, cmd = item.split(':', 1)
        out[int(idx)] = cmd.strip()
    return out


def main(argv=None) -> int:
    import rclpy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                              VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    class Mission(Node):
        def __init__(self):
            super().__init__('sim_mission')
            alt = self.declare_parameter('altitude_m', 2.5).value
            side = self.declare_parameter('side_m', 4.0).value
            faults = parse_faults(self.declare_parameter('faults', '').value)
            self.tol = self.declare_parameter('tolerance_m', 0.3).value
            self.hold_s = self.declare_parameter('hold_s', 2.0).value
            self.timeout_s = self.declare_parameter('timeout_s', 180.0).value
            lp_topic = self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1').value
            st_topic = self.declare_parameter('status_topic', '/fmu/out/vehicle_status_v1').value
            self.plan = square_plan(alt, side, faults)

            self.ocm_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
            self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
            self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
            self.fault_pub = self.create_publisher(String, '/sim/fault', 10)
            self.create_subscription(VehicleLocalPosition, lp_topic, self.on_lp, qos_profile_sensor_data)
            self.create_subscription(VehicleStatus, st_topic, self.on_status, qos_profile_sensor_data)
            self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected',
                                     self.on_land, qos_profile_sensor_data)
            self.lp = None
            self.status = None
            self.landed = None
            self.state = 'wait_position'
            self.state_t = time.monotonic()
            self.start_t = time.monotonic()
            self.wp_i = 0
            self.hold_since = None
            self.offboard_ticks = 0
            self.result = None
            self.create_timer(0.1, self.tick)

        def on_lp(self, m):
            self.lp = m

        def on_status(self, m):
            self.status = m

        def on_land(self, m):
            self.landed = m.landed

        def command(self, cmd, p1=0.0, p2=0.0, p7=0.0):
            m = VehicleCommand()
            m.timestamp = self.get_clock().now().nanoseconds // 1000
            m.command, m.param1, m.param2, m.param7 = cmd, float(p1), float(p2), float(p7)
            m.target_system, m.target_component = 1, 1
            m.source_system, m.source_component = 1, 1
            m.from_external = True
            self.cmd_pub.publish(m)

        def stream(self, wp: Waypoint):
            now_us = self.get_clock().now().nanoseconds // 1000
            o = OffboardControlMode()
            o.timestamp, o.position = now_us, True
            self.ocm_pub.publish(o)
            sp = TrajectorySetpoint()
            sp.timestamp = now_us
            sp.position = [float(wp.x), float(wp.y), float(wp.z)]
            sp.velocity = [math.nan] * 3
            sp.acceleration = [math.nan] * 3
            sp.jerk = [math.nan] * 3
            sp.yaw, sp.yawspeed = float(wp.yaw), math.nan
            self.sp_pub.publish(sp)

        def goto(self, state):
            self.get_logger().info(f'{self.state} -> {state}')
            self.state, self.state_t = state, time.monotonic()

        def finish(self, ok: bool, why: str):
            (self.get_logger().info if ok else self.get_logger().error)(f'mission {"done" if ok else "FAILED"}: {why}')
            self.result = 0 if ok else 1

        def tick(self):
            if self.result is not None:
                return
            if time.monotonic() - self.start_t > self.timeout_s:
                return self.finish(False, f'timeout in state {self.state}')
            lp, st = self.lp, self.status
            if self.state == 'wait_position':
                if lp and lp.xy_valid and lp.z_valid and lp.v_xy_valid:
                    # heading_good_for_control stays false with FRD-frame vision (yaw not
                    # north-aligned, ev_yaw_control.cpp); it only affects manual attitude mode.
                    self.goto('prestream')
                return
            first = self.plan.waypoints[0]
            if self.state == 'prestream':
                self.stream(first)
                self.offboard_ticks += 1
                if self.offboard_ticks >= 15:
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)  # custom main mode offboard
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                    self.goto('arming')
                return
            if self.state == 'arming':
                self.stream(first)
                if st and st.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                        st.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.goto('fly')
                elif time.monotonic() - self.state_t > 3.0:
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
                    self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                    self.state_t = time.monotonic()
                return
            if self.state == 'fly':
                if st and st.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    return self.finish(False, f'left offboard (nav_state {st.nav_state}); failsafe?')
                wp = self.plan.waypoints[self.wp_i]
                self.stream(wp)
                if lp and reached((lp.x, lp.y, lp.z), wp, self.tol):
                    if self.hold_since is None:
                        self.hold_since = time.monotonic()
                        self.get_logger().info(f'reached waypoint {self.wp_i}')
                        if wp.fault:
                            self.get_logger().warn(f'injecting fault "{wp.fault}"')
                            self.fault_pub.publish(String(data=wp.fault))
                    if time.monotonic() - self.hold_since >= self.hold_s:
                        self.hold_since = None
                        self.wp_i += 1
                        if self.wp_i >= len(self.plan.waypoints):
                            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                            self.goto('landing')
                return
            if self.state == 'landing':
                if st and st.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    return self.finish(True, 'landed and disarmed')

    rclpy.init(args=argv)
    node = Mission()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.result = 1
    result = node.result if node.result is not None else 1
    node.destroy_node()
    rclpy.try_shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
