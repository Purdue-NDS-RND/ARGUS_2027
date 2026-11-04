"""Mission feature flags read from the sequence YAML ``features:`` block."""

from __future__ import annotations

from typing import Any, Mapping, Optional


def get_feature_flag(features: Optional[Mapping[str, Any]], name: str) -> Any:
    """Return the value of flag ``name``, or False if it or the block is missing.

    Missions pass ``self.spec.features``. A missing key means the feature is off,
    so a sequence file does not need to list every flag.
    """
    if not features:
        return False
    return features.get(name, False)
