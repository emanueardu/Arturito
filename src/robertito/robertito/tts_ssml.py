import hashlib
import html
import re
from typing import List


_SENTENCE_SPLIT_RE = re.compile(r"(?<=[.!?])\s+")


def _split_sentences(text: str) -> List[str]:
    cleaned = " ".join(text.strip().split())
    if not cleaned:
        return []
    return [part.strip() for part in _SENTENCE_SPLIT_RE.split(cleaned) if part.strip()]


def _select_pause_ms(seed: str, short_ms: int, medium_ms: int) -> int:
    digest = hashlib.sha256(seed.encode("utf-8")).digest()
    return short_ms if digest[0] % 2 == 0 else medium_ms


def build_ssml(
    text: str,
    voice_name: str | None,
    rate_pct: float,
    pitch: str,
    short_ms: int,
    medium_ms: int,
    language: str = "es-AR",
) -> str:
    if not text:
        return ""
    sentences = _split_sentences(text)
    if not sentences:
        sentences = [text.strip()]

    chunks: List[str] = []
    for idx, sentence in enumerate(sentences):
        escaped = html.escape(sentence, quote=False)
        chunks.append(escaped)
        if idx < len(sentences) - 1:
            pause_ms = _select_pause_ms(sentence, short_ms, medium_ms)
            chunks.append(f"<break time=\"{pause_ms}ms\"/>")
    inner = " ".join(chunks)
    rate_str = f"{int(rate_pct * 100)}%"
    pitch_str = pitch or "0"
    speak_tag = f"<speak version=\"1.0\" xml:lang=\"{html.escape(language, quote=True)}\">"
    prosody = (
        f"<prosody rate=\"{rate_str}\" pitch=\"{html.escape(pitch_str, quote=True)}\">"
        f"{inner}"
        f"</prosody>"
    )
    if voice_name:
        voice = html.escape(voice_name, quote=True)
        return f"{speak_tag}<voice name=\"{voice}\">{prosody}</voice></speak>"
    return f"{speak_tag}{prosody}</speak>"
