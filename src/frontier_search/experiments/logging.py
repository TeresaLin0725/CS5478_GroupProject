"""JSONL event sink. Fixture events remain separate from experiment results."""

from dataclasses import asdict, is_dataclass
from enum import Enum
import json
from pathlib import Path
from typing import TextIO


def _encode(value: object) -> object:
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Enum):
        return value.value
    raise TypeError(f'cannot serialize {type(value).__name__}')


def dumps(value: object) -> str:
    return json.dumps(value, default=_encode, allow_nan=False, sort_keys=True)


class JsonlLogger:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = path.open('x', encoding='utf-8')

    def write(self, event: str, episode_id: str, **payload: object) -> None:
        self._file.write(dumps({'event': event, 'episode_id': episode_id, **payload}) + '\n')
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> 'JsonlLogger':
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
