"""Content hashing, shared by every stage that keys something on its inputs.

There is one way to reduce a Python value to a hashable payload in this project, and this is
it. That matters more than it looks: the failure this guards against is two callers computing
*different* digests for the *same* inputs and each quietly believing it has a cache miss — or,
worse, the same digest for different inputs. `docs/experiments/log.md` already records the
near-miss where two CLIs could have drifted into producing different ``design_hash`` values
from the same flags.

Extracted from :mod:`wheelopt.fea.cache` on 2026-08-10, unchanged, when the experiment store
needed the same reduction. `fea.cache` still owns the *policy* — which fields of an FEA
evaluation belong in its key, and the one named exclusion list that says why some do not.
This module owns only the mechanics.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Any

__all__ = ["content_digest", "plain"]


def plain(value: Any) -> Any:
    """Reduce a value to something ``json.dumps`` will order deterministically.

    Enums become their values, dataclasses become sorted dicts, and dicts are sorted by key,
    so the digest cannot depend on insertion order or on which process built the object.

    The float clause is not cosmetic. ``-0.0`` and ``0.0`` are equal, print differently, and
    would otherwise split a cache in two for a design that is identical — a spoke phase or a
    curvature arriving as a negative zero from one code path and a positive zero from another
    is exactly the kind of innocuous-looking difference this project keeps being bitten by.
    """
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value) and not isinstance(value, type):
        return {k: plain(v) for k, v in sorted(asdict(value).items())}
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, float):
        return value + 0.0
    return value


def content_digest(payload: Any, *, length: int = 16) -> str:
    """SHA-256 of ``payload`` reduced through :func:`plain`, truncated to ``length`` hex chars.

    Stable across processes and across dict insertion order. Sixteen hex characters is 64
    bits — at the ~10^4 evaluations this project plans, a collision is around 10^-11, and a
    key short enough to appear in a directory name and be read aloud is worth more than the
    remaining bits.
    """
    if length < 8 or length > 64:
        raise ValueError("length must be between 8 and 64 hex characters")
    blob = json.dumps(plain(payload), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()[:length]
