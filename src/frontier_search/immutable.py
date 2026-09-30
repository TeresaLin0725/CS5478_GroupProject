"""Reject mutable nested state instead of exposing mutable frozen dataclasses."""

from dataclasses import fields, is_dataclass
from enum import Enum
from math import isfinite


def require_immutable(value: object) -> None:
    if value is None or isinstance(value, (str, bytes, bool, int, Enum)):
        return
    if isinstance(value, float):
        if not isfinite(value):
            raise ValueError("contract numbers must be finite")
        return
    if isinstance(value, tuple):
        for item in value:
            require_immutable(item)
        return
    if is_dataclass(value) and value.__dataclass_params__.frozen:
        for field in fields(value):
            require_immutable(getattr(value, field.name))
        return
    raise TypeError(f"contract values must be deeply immutable, got {type(value).__name__}")


class ImmutableRecord:
    def __post_init__(self) -> None:
        require_immutable(self)
