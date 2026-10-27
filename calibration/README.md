# calibration

Versioned calibration deliverables (plan section 8). One directory per rig and calibration:

```text
calibration/rigs/<rig_id>/<YYYYMMDD>_<short_desc>/
  calibration.yaml        # index; validated by config/schemas/calibration.schema.json
  kalibr/                 # raw tool outputs (camchain, imu, report PDFs)
  vendor/                 # factory calibration reference
  validation.md           # independent validation recording + results
```

Rules:

- Transform convention: `T_A_B` maps coordinates from B into A. Units: m, s, rad.
- Intrinsics per operating resolution/crop. Lock focus; document exposure/gain.
- Include camera-IMU spatial + temporal calibration, IMU noise/random walk/saturation, `T_base_imu`,
  and GNSS lever arm if used.
- Never edit a calibration in place. Remounts, impacts or a second stereo unit produce a new directory.
- Calibration recordings (bags) stay outside git; reference them by session_id.
