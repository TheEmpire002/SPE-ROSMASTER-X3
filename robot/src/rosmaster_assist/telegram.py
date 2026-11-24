"""Send messages to the configured Telegram chat."""

from __future__ import annotations

import json
import logging
import os
import threading
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from . import config

log = logging.getLogger(__name__)


class TelegramError(Exception):
    """A message could not be sent."""


_state_lock = threading.RLock()


def _read_state() -> dict:
    try:
        with config.TELEGRAM_STATE_PATH.open(encoding="utf-8") as source:
            data = json.load(source)
        if (not isinstance(data, dict) or not isinstance(data.get("paused"), bool)
                or not isinstance(data.get("offset"), int) or data["offset"] < 0):
            raise ValueError("Invalid Telegram state")
        return data
    except FileNotFoundError:
        return {"paused": False, "offset": 0}
    except (OSError, ValueError):
        raise TelegramError("Telegram-Zustand konnte nicht gelesen werden") from None


def _write_state(state: dict) -> None:
    path = config.TELEGRAM_STATE_PATH
    temporary = path.with_suffix(".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(json.dumps(state), encoding="utf-8")
        os.replace(temporary, path)
    except OSError:
        raise TelegramError("Telegram-Zustand konnte nicht gespeichert werden") from None


def _secret_data() -> dict:
    try:
        with config.TELEGRAM_SECRETS_PATH.open(encoding="utf-8") as source:
            data = json.load(source)
        if not isinstance(data, dict):
            raise ValueError("Invalid credentials")
    except FileNotFoundError:
        data = {}
    except (OSError, ValueError):
        raise TelegramError("Telegram-Konfiguration konnte nicht gelesen werden") from None
    return data


def _credentials() -> tuple[str, str]:
    data = _secret_data()
    token = os.environ.get("TELEGRAM_BOT_TOKEN") or data.get("bot_token")
    chat_id = os.environ.get("TELEGRAM_CHAT_ID") or data.get("chat_id")
    if not isinstance(token, str) or not token.strip() or not isinstance(chat_id, (str, int)):
        raise TelegramError("Telegram ist nicht eingerichtet")
    chat_id = str(chat_id).strip()
    if not chat_id or not token.strip().replace(":", "").replace("_", "").replace("-", "").isalnum():
        raise TelegramError("Telegram-Konfiguration ist ungültig")
    return token.strip(), chat_id


def _allowed_users() -> set[int]:
    configured = os.environ.get("TELEGRAM_ALLOWED_USER_IDS")
    values = ([value for value in configured.split(",") if value.strip()]
              if configured is not None else _secret_data().get("allowed_user_ids", []))
    if not isinstance(values, list):
        raise TelegramError("Telegram-Benutzerliste ist ungültig")
    try:
        if any(isinstance(value, bool) or not str(value).strip().isdigit() for value in values):
            raise ValueError("Invalid user ID")
        return {int(value) for value in values if int(value) > 0}
    except (TypeError, ValueError):
        raise TelegramError("Telegram-Benutzerliste ist ungültig") from None


def send_message(text: str, *, markdown: bool = False) -> None:
    """Send text or standard Markdown to the configured chat unless paused."""
    with _state_lock:
        if _read_state()["paused"]:
            raise TelegramError("Telegram-Nachrichten sind pausiert")
        _send_message(text, markdown=markdown)


def _send_message(text: str, *, markdown: bool = False) -> None:
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


def _request(method: str, payload: dict) -> dict:
    token, _ = _credentials()
    request = Request(
        f"https://api.telegram.org/bot{token}/{method}",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urlopen(request, timeout=config.TELEGRAM_TIMEOUT_S) as response:
            data = json.load(response)
        if not isinstance(data, dict) or data.get("ok") is not True:
            raise TelegramError("Telegram hat die Anfrage abgelehnt")
        return data
    except HTTPError as exc:
        raise TelegramError(f"Telegram hat die Anfrage abgelehnt (HTTP {exc.code})") from None
    except (URLError, OSError, ValueError):
        raise TelegramError("Telegram ist nicht erreichbar") from None


class CommandService:
    """Poll commands on a background thread without affecting robot control."""

    def __init__(self) -> None:
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._username: str | None = None
        self._bot_id: str | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="telegram-commands", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=config.TELEGRAM_TIMEOUT_S + 1.0)
            if not self._thread.is_alive():
                self._thread = None

    def _loop(self) -> None:
        last_error = None
        while not self._stop.is_set():
            try:
                if _allowed_users():
                    self._poll()
                last_error = None
            except TelegramError as exc:
                if str(exc) != last_error:
                    log.warning("Telegram commands unavailable: %s", exc)
                    last_error = str(exc)
            except Exception:
                if last_error != "unexpected":
                    log.warning("Telegram commands unavailable due to an unexpected error")
                    last_error = "unexpected"
            self._stop.wait(config.TELEGRAM_COMMAND_POLL_S)

    def _poll(self) -> None:
        token, chat_id = _credentials()
        allowed = _allowed_users()
        if not allowed:
            return
        bot_id = token.split(":", 1)[0]
        if self._bot_id != bot_id:
            self._username = None
        if self._username is None:
            bot = _request("getMe", {}).get("result")
            if not isinstance(bot, dict) or not isinstance(bot.get("username"), str):
                raise TelegramError("Telegram-Botname konnte nicht gelesen werden")
            self._username = bot["username"].lower()
            self._bot_id = bot_id
        if self._stop.is_set():
            return
        with _state_lock:
            state = _read_state()
            offset = state["offset"] if state.get("bot_id") == bot_id else 0
        updates = _request("getUpdates", {"offset": offset, "timeout": 0, "allowed_updates": ["message"]}).get("result")
        if not isinstance(updates, list):
            raise TelegramError("Telegram-Befehle konnten nicht gelesen werden")
        for update in updates:
            if self._stop.is_set():
                break
            self._handle_update(update, bot_id, chat_id, allowed)

    def _handle_update(self, update: object, bot_id: str, chat_id: str, allowed: set[int]) -> None:
        if not isinstance(update, dict) or type(update.get("update_id")) is not int or update["update_id"] < 0:
            return
        message = update.get("message")
        message = message if isinstance(message, dict) else {}
        chat = message.get("chat") if isinstance(message.get("chat"), dict) else {}
        sender = message.get("from") if isinstance(message.get("from"), dict) else {}
        text = message.get("text")
        command = text.strip().split(maxsplit=1)[0] if isinstance(text, str) and text.strip() else ""
        name, separator, addressed_to = command.partition("@")
        authorized = (str(chat.get("id")) == chat_id and type(sender.get("id")) is int
                      and sender["id"] in allowed and not sender.get("is_bot")
                      and (not separator or addressed_to.lower() == self._username))
        response = None
        with _state_lock:
            state = _read_state()
            previous_offset = state["offset"] if state.get("bot_id") == bot_id else 0
            if update["update_id"] < previous_offset:
                return
            state["bot_id"] = bot_id
            state["offset"] = update["update_id"] + 1
            if authorized and name == config.TELEGRAM_PAUSE_COMMAND:
                state["paused"] = True
                response = "Telegram-Nachrichten pausiert. Mit /resume wieder aktivieren."
            elif authorized and name == config.TELEGRAM_RESUME_COMMAND:
                state["paused"] = False
                response = "Telegram-Nachrichten wieder aktiviert."
            elif authorized and name == config.TELEGRAM_STATUS_COMMAND:
                response = "Telegram-Nachrichten sind pausiert." if state["paused"] else "Telegram-Nachrichten sind aktiv."
            _write_state(state)
        if response is not None:
            try:
                _send_message(response)
            except TelegramError as exc:
                log.warning("Telegram command confirmation failed: %s", exc)