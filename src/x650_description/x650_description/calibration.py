"""Calibration loader (M1/M2).

Loads calibration/rigs/<rig_id>/<calibration_id>/calibration.yaml, validates it against
config/schemas/calibration.schema.json, verifies its content hash and produces static transforms.

Convention: T_A_B maps coordinates from frame B into frame A. Units: m, s, rad.

Hash: sha256 over every file in the calibration directory (sorted relative paths), where
calibration.yaml contributes its canonical JSON with the `calibration_hash` field removed. A
stored hash that does not match means the calibration was edited in place; loading fails.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import yaml

INDEX = 'calibration.yaml'


class CalibrationError(RuntimeError):
    pass


@dataclass(frozen=True)
class StaticTransform:
    parent: str
    child: str
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


def find_schema_dir(start: Path) -> Path | None:
    for d in [start, *start.parents]:
        cand = d / 'config' / 'schemas'
        if (cand / 'calibration.schema.json').exists():
            return cand
    return None


def compute_hash(calib_dir) -> str:
    calib_dir = Path(calib_dir)
    h = hashlib.sha256()
    for f in sorted(p for p in calib_dir.rglob('*') if p.is_file()):
        rel = f.relative_to(calib_dir).as_posix()
        if rel == INDEX:
            data = yaml.safe_load(f.read_text())
            data.pop('calibration_hash', None)
            content = json.dumps(data, sort_keys=True, separators=(',', ':')).encode()
        else:
            content = f.read_bytes()
        h.update(rel.encode() + b'\0' + hashlib.sha256(content).digest())
    return 'sha256:' + h.hexdigest()


def validate(data: dict, schema_dir: Path) -> None:
    from jsonschema import Draft202012Validator
    schema = json.loads((schema_dir / 'calibration.schema.json').read_text())
    errors = sorted(Draft202012Validator(schema).iter_errors(data), key=lambda e: list(e.path))
    if errors:
        msg = '; '.join(f"{'/'.join(map(str, e.path)) or '<root>'}: {e.message}" for e in errors)
        raise CalibrationError(f'schema validation failed: {msg}')


def _check_quat(q, where):
    n = sum(v * v for v in q) ** 0.5
    if abs(n - 1.0) > 1e-6:
        raise CalibrationError(f'{where}: rotation_xyzw not unit norm ({n:.9f})')


def load(path, schema_dir=None, verify_hash=True) -> dict:
    """Load and check a calibration. `path` is calibration.yaml or its directory."""
    path = Path(path)
    calib_dir = path if path.is_dir() else path.parent
    data = yaml.safe_load((calib_dir / INDEX).read_text())
    schema_dir = Path(schema_dir) if schema_dir else find_schema_dir(calib_dir.resolve())
    if schema_dir is None:
        raise CalibrationError('config/schemas not found; pass schema_dir explicitly')
    validate(data, schema_dir)
    _check_quat(data['T_base_imu']['rotation_xyzw'], 'T_base_imu')
    ids = [c['camera_id'] for c in data['cameras']]
    if len(set(ids)) != len(ids):
        raise CalibrationError(f'duplicate camera_id in {ids}')
    for c in data['cameras']:
        _check_quat(c['T_imu_cam']['rotation_xyzw'], f'{c["camera_id"]}.T_imu_cam')
        if len(c['intrinsics']) != 4:
            raise CalibrationError(f'{c["camera_id"]}: intrinsics must be fx, fy, cx, cy')
    actual = compute_hash(calib_dir)
    stored = data.get('calibration_hash')
    if verify_hash:
        if not stored:
            raise CalibrationError(f'calibration_hash missing; computed {actual}')
        if stored != actual:
            raise CalibrationError(
                f'calibration_hash mismatch (stored {stored}, computed {actual}). '
                'Calibrations are never edited in place: create a new calibration directory.')
    data['_dir'] = str(calib_dir)
    data['_computed_hash'] = actual
    return data


def static_transforms(calib: dict, base_frame='base_link', imu_frame='imu_link') -> list[StaticTransform]:
    """base_link -> imu_link and imu_link -> <camera_id>_optical, as TF parent -> child.

    A TF from parent P to child C carries T_P_C (it maps child coordinates into the parent)."""
    def st(parent, child, T):
        return StaticTransform(parent, child, tuple(map(float, T['translation'])),
                               tuple(map(float, T['rotation_xyzw'])))
    out = [st(base_frame, imu_frame, calib['T_base_imu'])]
    for c in calib['cameras']:
        out.append(st(imu_frame, f'{c["camera_id"]}_optical', c['T_imu_cam']))
    return out


def main(argv=None) -> int:
    """calibration_hash <dir> [--write]: print (or fill in) a new calibration's hash."""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('calibration_dir')
    ap.add_argument('--write', action='store_true',
                    help='set calibration_hash in a NEW calibration.yaml that has none yet')
    args = ap.parse_args(argv)
    d = Path(args.calibration_dir)
    h = compute_hash(d)
    if args.write:
        idx = d / INDEX
        data = yaml.safe_load(idx.read_text())
        if data.get('calibration_hash') and data['calibration_hash'] != h:
            raise SystemExit('refusing to overwrite an existing, different hash (edited in place?)')
        text = idx.read_text()
        if 'calibration_hash' not in data:
            idx.write_text(f'calibration_hash: "{h}"\n' + text)
    print(h)
    return 0
