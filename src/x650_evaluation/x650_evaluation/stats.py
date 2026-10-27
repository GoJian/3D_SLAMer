"""Small statistics helpers for machine-readable reports."""
from __future__ import annotations

import numpy as np


def percentiles(values, ps=(50, 95, 99)) -> dict:
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return {f'p{p}': None for p in ps} | {'n': 0}
    out = {f'p{p}': float(np.percentile(v, p)) for p in ps}
    out.update(n=int(v.size), mean=float(v.mean()), max=float(v.max()), min=float(v.min()))
    return out
