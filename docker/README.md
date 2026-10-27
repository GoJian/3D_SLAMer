# docker

Unverified starting points. Pin base images and versions during M0 / M5, then remove this note.

- `jetson/` — onboard stack (JetPack L4T base + ROS 2 + ZED SDK). The ZED SDK and capture driver
  usually need host kernel modules; the container only provides userspace.
- `workstation/` — post-flight reconstruction (COLMAP + OpenMVS), kept separate from flight software.
