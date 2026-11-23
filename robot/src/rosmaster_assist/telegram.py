"""Send messages to the configured Telegram chat."""

from __future__ import annotations

import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from . import config


class TelegramError(Exception):
    """A message could not be sent."""


def _credentials() -> tuple[str, str]:
    try:
        with config.TELEGRAM_SECRETS_PATH.open(encoding="utf-8") as source:
            data = json.load(source)
        if not isinstance(data, dict):
            raise ValueError("Invalid credentials")
    except FileNotFoundError:
        data = {}
    except (OSError, ValueError):
        raise TelegramError("Telegram-Konfiguration konnte nicht gelesen werden") from None

    token = os.environ.get("TELEGRAM_BOT_TOKEN") or data.get("bot_token")
    chat_id = os.environ.get("TELEGRAM_CHAT_ID") or data.get("chat_id")
    if not isinstance(token, str) or not token.strip() or not isinstance(chat_id, (str, int)):
        raise TelegramError("Telegram ist nicht eingerichtet")
    chat_id = str(chat_id).strip()
    if not chat_id or not token.strip().replace(":", "").replace("_", "").replace("-", "").isalnum():
        raise TelegramError("Telegram-Konfiguration ist ungültig")
    return token.strip(), chat_id


def send_message(text: str, *, markdown: bool = False) -> None:
    """Send text or standard Markdown to the configured chat."""
    if not isinstance(text, str) or not text.strip():
        raise TelegramError("Nachricht ist leer")
    if not isinstance(markdown, bool):
        raise TelegramError("Ungültiges Nachrichtenformat")
    try:
        length = len(text.encode("utf-16-le")) // 2
    except UnicodeError:
        raise TelegramError("Nachricht enthält ungültige Zeichen") from None
    if length > config.TELEGRAM_MAX_MESSAGE_UTF16:
        raise TelegramError(f"Nachricht ist zu lang (höchstens {config.TELEGRAM_MAX_MESSAGE_UTF16} Zeichen)")
    token, chat_id = _credentials()
    payload = {"chat_id": chat_id, "text": text}
    if markdown:
        try:
            from telegramify_markdown import convert
        except ImportError:
            raise TelegramError("Markdown-Unterstützung fehlt auf dem Roboter") from None
        rendered, entities = convert(text)
        if not rendered.strip():
            raise TelegramError("Nachricht ist leer")
        if len(rendered.encode("utf-16-le")) // 2 > config.TELEGRAM_MAX_MESSAGE_UTF16:
            raise TelegramError(f"Nachricht ist zu lang (höchstens {config.TELEGRAM_MAX_MESSAGE_UTF16} Zeichen)")
        payload["text"] = rendered
        payload["entities"] = [entity.to_dict() for entity in entities]
    request = Request(
        f"https://api.telegram.org/bot{token}/sendMessage",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=config.TELEGRAM_TIMEOUT_S) as response:
            result = json.load(response)
        if not isinstance(result, dict) or result.get("ok") is not True:
            raise TelegramError("Telegram hat die Nachricht abgelehnt")
    except HTTPError as exc:
        raise TelegramError(f"Telegram hat die Nachricht abgelehnt (HTTP {exc.code})") from None
    except (URLError, OSError, ValueError):
        raise TelegramError("Telegram ist nicht erreichbar") from None