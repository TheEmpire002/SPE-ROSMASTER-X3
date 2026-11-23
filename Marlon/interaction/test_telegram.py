"""Tests for Telegram message delivery without network access."""

import asyncio
import io
import json
import pathlib
import sys
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import telegram  # noqa: E402


class TelegramTests(unittest.TestCase):
    def setUp(self):
        self.environment = patch.dict("os.environ", {"TELEGRAM_BOT_TOKEN": "123:secret", "TELEGRAM_CHAT_ID": "-42"})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.secrets = patch.object(telegram.config, "TELEGRAM_SECRETS_PATH", pathlib.Path("does-not-exist.json"))
        self.secrets.start()
        self.addCleanup(self.secrets.stop)

    @patch("rosmaster_assist.telegram.urlopen")
    def test_plain_text_request(self, open_url):
        open_url.return_value.__enter__.return_value = io.BytesIO(b'{"ok": true}')
        telegram.send_message("Hallo")
        request = open_url.call_args.args[0]
        self.assertEqual(json.loads(request.data), {"chat_id": "-42", "text": "Hallo"})
        self.assertEqual(open_url.call_args.kwargs["timeout"], telegram.config.TELEGRAM_TIMEOUT_S)

    @patch("rosmaster_assist.telegram.urlopen")
    def test_markdown_is_converted_to_telegram_entities(self, open_url):
        open_url.return_value.__enter__.return_value = io.BytesIO(b'{"ok": true}')
        telegram.send_message("**Hallo** Welt.", markdown=True)
        data = json.loads(open_url.call_args.args[0].data)
        self.assertEqual(data["text"], "Hallo Welt.")
        self.assertEqual(data["entities"][0]["type"], "bold")
        self.assertNotIn("parse_mode", data)

    @patch("rosmaster_assist.telegram.urlopen")
    def test_invalid_messages_do_not_contact_telegram(self, open_url):
        for text in ("", "  ", "x" * 4097, "\U0001f600" * 2049, "\ud800"):
            with self.subTest(text=text[:5]), self.assertRaises(telegram.TelegramError):
                telegram.send_message(text)
        open_url.assert_not_called()

    @patch("rosmaster_assist.telegram.urlopen")
    def test_errors_do_not_expose_token(self, open_url):
        for error in (URLError("123:secret"), HTTPError("https://123:secret", 401, "bad", {}, None)):
            open_url.side_effect = error
            with self.assertRaises(telegram.TelegramError) as raised:
                telegram.send_message("Hallo")
            self.assertNotIn("123:secret", str(raised.exception))

    def test_missing_credentials(self):
        with patch.dict("os.environ", {}, clear=True), self.assertRaises(telegram.TelegramError):
            telegram.send_message("Hallo")

    def test_credentials_file_and_environment_override(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "secrets.json"
            path.write_text(json.dumps({"bot_token": "123:file", "chat_id": -99}), encoding="utf-8")
            with patch.object(telegram.config, "TELEGRAM_SECRETS_PATH", path):
                with patch.dict("os.environ", {}, clear=True):
                    self.assertEqual(telegram._credentials(), ("123:file", "-99"))
                self.assertEqual(telegram._credentials(), ("123:secret", "-42"))


class TelegramEndpointTests(unittest.TestCase):
    def test_endpoint_validates_and_runs_sender(self):
        from rosmaster_assist.api.server import create_app
        from rosmaster_assist.drive import MockDriver

        app = create_app(MockDriver(), use_mock_camera=True)

        async def post(body, content_type=b"application/json"):
            messages = []
            received = False

            async def receive():
                nonlocal received
                if received:
                    return {"type": "http.disconnect"}
                received = True
                return {"type": "http.request", "body": body, "more_body": False}

            async def send(message):
                messages.append(message)

            await app({"type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1",
                       "method": "POST", "scheme": "http", "path": "/telegram/send",
                       "raw_path": b"/telegram/send", "query_string": b"",
                       "headers": [(b"content-type", content_type)],
                       "client": ("127.0.0.1", 1234), "server": ("localhost", 8000)}, receive, send)
            return messages[0]["status"], json.loads(messages[1]["body"])

        with patch("rosmaster_assist.telegram.send_message") as send_message:
            self.assertEqual(asyncio.run(post(b'{"text":"hello","markdown":true}')), (200, {"ok": True}))
            send_message.assert_called_once_with("hello", markdown=True)
            self.assertEqual(asyncio.run(post(b'{"text":"hello","markdown":"yes"}'))[0], 400)
            self.assertEqual(asyncio.run(post(b'not json'))[0], 400)
            self.assertEqual(asyncio.run(post(b'{"text":"hello"}', b"text/plain"))[0], 415)
            send_message.assert_called_once()
            send_message.side_effect = telegram.TelegramError("Telegram ist nicht erreichbar")
            status, body = asyncio.run(post(b'{"text":"hello"}'))
            self.assertEqual((status, body), (400, {"fehler": "Telegram ist nicht erreichbar"}))


if __name__ == "__main__":
    unittest.main()