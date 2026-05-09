"""Pool de frases TTS con anti-repetición simple."""
from __future__ import annotations

import random
from typing import Optional


class TtsPool:
    """Wrapper sobre dict[category, list[str]] con anti-repetición."""

    def __init__(self, pools: dict[str, list[str]]) -> None:
        self._pools: dict[str, list[str]] = {
            k: list(v) for k, v in (pools or {}).items() if isinstance(v, list)
        }
        self._last_used: dict[str, str] = {}

    def sample(self, category: str) -> Optional[str]:
        """Devuelve una frase random de la pool del category, o None si vacía."""
        pool = self._pools.get(category)
        if not pool:
            return None
        if len(pool) == 1:
            phrase = pool[0]
            self._last_used[category] = phrase
            return phrase
        last = self._last_used.get(category)
        choices = [p for p in pool if p != last] or pool
        phrase = random.choice(choices)
        self._last_used[category] = phrase
        return phrase

    def has(self, category: str) -> bool:
        return bool(self._pools.get(category))
