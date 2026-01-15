import hashlib
import re
from typing import List


_CONFIRMATIONS = [
    "Entendido.",
    "Listo.",
    "En curso.",
    "De acuerdo.",
]

_ACTION_PROMPTS = [
    "Deseas que continue?",
    "Confirmo el siguiente paso?",
    "Algo mas que deba revisar?",
]

_QUESTION_PROMPTS = [
    "Quieres que lo detalle?",
    "Confirmo la ejecucion?",
    "Quieres que lo aplique?",
]

_SENTENCE_SPLIT_RE = re.compile(r"(?<=[.!?])\s+")


def _stable_choice(seed: str, options: List[str]) -> str:
    if not options:
        return ""
    digest = hashlib.sha256(seed.encode("utf-8")).digest()
    idx = digest[0] % len(options)
    return options[idx]


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
    """Return a concise, structured response for cabin-style speech."""
    if not text:
        return ""

    sentences = _split_sentences(text)
    if not sentences:
        sentences = [text.strip()]

    confirmation = _stable_choice(text, _CONFIRMATIONS)
    main_sentences = sentences[:2]

    has_question = any("?" in sentence for sentence in sentences)
    if has_question:
        prompt = _stable_choice(text + "?", _QUESTION_PROMPTS)
    else:
        prompt = _stable_choice(text + "!", _ACTION_PROMPTS)

    extra = ""
    if len(sentences) > 2:
        extra = f"Siguiente: {sentences[2].strip()}"

    output_sentences = [confirmation]
    output_sentences.extend(_ensure_terminal_punct(s) for s in main_sentences)
    if extra:
        output_sentences.append(_ensure_terminal_punct(extra))
    if prompt:
        output_sentences.append(_ensure_terminal_punct(prompt))

    return " ".join(s for s in output_sentences if s)
