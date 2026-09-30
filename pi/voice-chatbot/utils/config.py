"""Configuration loader. Reads config.yaml and resolves ${ENV_VAR} placeholders."""

import os
import re
from pathlib import Path
from typing import Any

import yaml


_ENV_RE = re.compile(r"\$\{(\w+)\}")


class Config:
    """Loads a YAML config file with environment variable substitution.

    Usage:
        config = Config("config.yaml")
        api_key = config.get("asr.api_key")
        threshold = config.get("vad.silence_threshold_ms", 800)
    """

    def __init__(self, path: str):
        raw_text = Path(path).read_text(encoding="utf-8")

        def _resolve_env(match: re.Match) -> str:
            var = match.group(1)
            value = os.environ.get(var)
            if value is None:
                raise ValueError(
                    f"Environment variable '{var}' referenced in config "
                    f"but not set. Required by: {match.string[match.start():match.end()+20]}..."
                )
            return value

        resolved = _ENV_RE.sub(_resolve_env, raw_text)
        self._data: dict[str, Any] = yaml.safe_load(resolved)

    def get(self, key: str, default: Any = None) -> Any:
        """Get a value by dotted path, e.g. 'vad.silence_threshold_ms'."""
        parts = key.split(".")
        node = self._data
        for part in parts:
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return default
        return node

    def __getitem__(self, key: str) -> Any:
        result = self.get(key)
        if result is None:
            raise KeyError(key)
        return result
