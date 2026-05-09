import re
from typing import List


_SENTENCE_SPLIT_RE = re.compile(r"(?<=[.!?])\s+")


def _split_sentences(text: str) -> List[str]:
    cleaned = " ".join(text.strip().split())
    if not cleaned:
        return []
    parts = _SENTENCE_SPLIT_RE.split(cleaned)
    sentences: List[str] = []
    for part in parts:
        part = part.strip()
        if not part:
            continue
        sentences.append(part)
    return sentences


def _ensure_terminal_punct(sentence: str) -> str:
    if not sentence:
        return sentence
    if sentence[-1] in ".!?":
        return sentence
    return sentence + "."


def format_jarvis_style(text: str) -> str:
    """Return a concise spoken response without adding extra trailing prompts."""
    if not text:
        return ""

    sentences = _split_sentences(text)
    if not sentences:
        sentences = [text.strip()]

    main_sentences = sentences[:2]

    extra = ""
    if len(sentences) > 2:
        extra = f"Siguiente: {sentences[2].strip()}"

    output_sentences = [_ensure_terminal_punct(s) for s in main_sentences]
    if extra:
        output_sentences.append(_ensure_terminal_punct(extra))

    return " ".join(s for s in output_sentences if s)
