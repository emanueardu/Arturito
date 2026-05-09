from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_LOGGER = logging.getLogger(__name__)


class FaceDatabase:
    def __init__(self, db_path: str) -> None:
        self._path = Path(db_path).expanduser()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._faces: Dict[str, np.ndarray] = {}
        self.load()

    def load(self) -> None:
        if not self._path.exists():
            return
        try:
            with self._path.open("r", encoding="utf-8") as fd:
                data = json.load(fd)
        except (json.JSONDecodeError, OSError) as exc:
            _LOGGER.warning("Unable to load face database: %s", exc)
            return
        entries = data.get("people", {})
        for name, entry in entries.items():
            embedding = entry.get("embedding")
            if not embedding:
                continue
            try:
                self._faces[name] = np.asarray(embedding, dtype=float)
            except (TypeError, ValueError):
                _LOGGER.warning("Ignoring invalid embedding for %s", name)

    def save(self) -> None:
        payload: Dict[str, Dict[str, List[float]]] = {"people": {}}
        for name, embedding in self._faces.items():
            payload["people"][name] = {"embedding": embedding.tolist()}
        try:
            with self._path.open("w", encoding="utf-8") as fd:
                json.dump(payload, fd, indent=2)
        except OSError as exc:
            _LOGGER.error("Failed to write face database: %s", exc)

    def update_person(self, name: str, embedding: np.ndarray) -> None:
        self._faces[name] = np.asarray(embedding, dtype=float)
        self.save()

    def entries(self) -> Dict[str, np.ndarray]:
        return dict(self._faces)

    def has_person(self, name: str) -> bool:
        return name in self._faces

    def find_best_match(
        self, embedding: np.ndarray, threshold: float = 0.6
    ) -> Tuple[Optional[str], Optional[float]]:
        if not self._faces:
            return None, None
        target = np.asarray(embedding, dtype=float)
        results = {
            name: float(np.linalg.norm(target - data))
            for name, data in self._faces.items()
        }
        best_name = min(results, key=results.get)
        best_distance = results[best_name]
        if best_distance <= threshold:
            return best_name, best_distance
        return None, best_distance
