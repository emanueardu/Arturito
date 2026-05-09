"""Provider de ElevenLabs TTS para voice_synth_node.

Sintetiza texto a audio MP3 via API HTTP, lo decodifica a PCM con pydub,
y lo expone como bytes WAV reproducibles por sounddevice.

Cache local por hash(text + voice_id + settings) para evitar pagar 2 veces
la misma frase. Cache compartida con Piper en /home/robot/.cache/robertito/voice/.

Si la API falla (sin internet, key invalida, rate limit), levanta excepcion
y el voice_synth_node hace fallback a Piper.
"""
from __future__ import annotations

import hashlib
import io
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import requests


_API_URL = "https://api.elevenlabs.io/v1/text-to-speech/{voice_id}"
_DEFAULT_MODEL = "eleven_multilingual_v2"
_TIMEOUT_S = 8.0


@dataclass(frozen=True)
class VoiceSettings:
    """Settings de voz por categoria emocional."""
    stability: float = 0.5
    similarity_boost: float = 0.75
    style: float = 0.5
    use_speaker_boost: bool = True

    def to_dict(self) -> dict:
        return {
            "stability": self.stability,
            "similarity_boost": self.similarity_boost,
            "style": self.style,
            "use_speaker_boost": self.use_speaker_boost,
        }


class ElevenLabsProvider:
    """Sintetizador de texto a WAV via ElevenLabs API."""

    def __init__(
        self,
        api_key: str,
        voice_id: str,
        model_id: str = _DEFAULT_MODEL,
        cache_dir: Optional[Path] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if not api_key:
            raise ValueError("ElevenLabs API key vacia")
        if not voice_id:
            raise ValueError("ElevenLabs voice_id vacio")
        self._api_key = api_key
        self._voice_id = voice_id
        self._model_id = model_id
        self._cache_dir = cache_dir or Path.home() / ".cache" / "robertito" / "voice"
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._logger = logger or logging.getLogger(__name__)

    def _cache_key(self, text: str, settings: VoiceSettings) -> str:
        """Hash determinista del texto + voz + settings."""
        h = hashlib.sha256()
        h.update(text.encode("utf-8"))
        h.update(self._voice_id.encode("utf-8"))
        h.update(self._model_id.encode("utf-8"))
        h.update(repr(settings.to_dict()).encode("utf-8"))
        return h.hexdigest()

    def _cache_path(self, key: str) -> Path:
        return self._cache_dir / f"el_{key}.wav"

    def synthesize(
        self,
        text: str,
        settings: Optional[VoiceSettings] = None,
    ) -> bytes:
        """Sintetiza texto a WAV bytes (PCM 22050 Hz mono int16).

        Devuelve los bytes del WAV listos para sounddevice o cualquier player.
        Lanza excepcion si la API falla; voice_synth puede catchear y caer a Piper.
        """
        settings = settings or VoiceSettings()
        cache_key = self._cache_key(text, settings)
        cache_path = self._cache_path(cache_key)

        # Cache hit
        if cache_path.exists() and cache_path.stat().st_size > 1000:
            self._logger.debug(f"elevenlabs cache hit: {cache_key[:12]}")
            return cache_path.read_bytes()

        # Cache miss: llamar a la API
        self._logger.info(f"elevenlabs synth: voice={self._voice_id[:8]} text={text[:40]!r}")
        url = _API_URL.format(voice_id=self._voice_id)
        headers = {
            "xi-api-key": self._api_key,
            "Content-Type": "application/json",
            "Accept": "audio/mpeg",
        }
        body = {
            "text": text,
            "model_id": self._model_id,
            "voice_settings": settings.to_dict(),
        }
        resp = requests.post(url, headers=headers, json=body, timeout=_TIMEOUT_S)
        if resp.status_code != 200:
            raise RuntimeError(
                f"ElevenLabs API HTTP {resp.status_code}: {resp.text[:200]}"
            )
        mp3_bytes = resp.content
        if len(mp3_bytes) < 1000:
            raise RuntimeError(f"ElevenLabs respuesta sospechosamente chica: {len(mp3_bytes)}b")

        # Convertir MP3 -> WAV con pydub
        try:
            from pydub import AudioSegment
        except ImportError as exc:
            raise RuntimeError("pydub no instalado: pip install pydub") from exc

        audio = AudioSegment.from_mp3(io.BytesIO(mp3_bytes))
        # Normalizar a 22050 Hz mono int16 (compatible con voice_synth y Piper)
        audio = audio.set_frame_rate(22050).set_channels(1).set_sample_width(2)
        wav_buf = io.BytesIO()
        audio.export(wav_buf, format="wav")
        wav_bytes = wav_buf.getvalue()

        # Guardar en cache
        try:
            cache_path.write_bytes(wav_bytes)
            self._logger.debug(f"elevenlabs cache save: {cache_path}")
        except OSError as exc:
            self._logger.warning(f"no pude guardar cache {cache_path}: {exc}")

        return wav_bytes


def load_api_key_from_file(path: str) -> Optional[str]:
    """Lee ELEVENLABS_API_KEY de un archivo .env simple."""
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    key, _, value = line.partition("=")
                    if key.strip() == "ELEVENLABS_API_KEY":
                        return value.strip().strip('"').strip("'")
    except OSError:
        pass
    return None
