#!/usr/bin/env python3
"""Validate config/*.yaml instances against config/schemas. Requires: pip install jsonschema pyyaml"""
import json
import sys
from pathlib import Path

import yaml
from jsonschema import Draft202012Validator

try:  # jsonschema >= 4.18
    from referencing import Registry, Resource
except ImportError:  # Ubuntu 24.04 apt ships jsonschema 4.10
    Registry = Resource = None

ROOT = Path(__file__).resolve().parents[1]
SCHEMAS = ROOT / "config" / "schemas"

PAIRS = {
    "config/hardware/*.yaml": "hardware_revision.schema.json",
    "config/clock_policy.yaml": "clock_policy.schema.json",
    "config/dependencies.*.lock.yaml": "dependency_lock.schema.json",
    "calibration/rigs/*/*/calibration.yaml": "calibration.schema.json",
}


def main() -> int:
    schemas = {p.name: json.loads(p.read_text()) for p in SCHEMAS.glob("*.json")}
    kwargs = {}
    if Registry is not None:
        kwargs['registry'] = Registry().with_resources(
            (name, Resource.from_contents(s)) for name, s in schemas.items()
        )
    for s in schemas.values():
        Draft202012Validator.check_schema(s)
    failed = 0
    for pattern, schema_name in PAIRS.items():
        validator = Draft202012Validator(schemas[schema_name], **kwargs)
        for path in sorted(ROOT.glob(pattern)):
            errors = list(validator.iter_errors(yaml.safe_load(path.read_text())))
            status = "FAIL" if errors else "ok"
            print(f"{status:4} {path.relative_to(ROOT)}")
            for e in errors:
                print(f"     {'/'.join(map(str, e.path))}: {e.message}")
            failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
