(() => {
  const element = (id) => document.getElementById(id);
  const dialog = element("telegram-dialog");
  const text = element("telegram-text");
  const file = element("telegram-datei");
  const markdown = element("telegram-markdown");
  const send = element("telegram-senden");
  const message = element("telegram-meldung");

  function showMessage(value, isError = false) {
    message.textContent = value;
    message.classList.toggle("fehler", isError);
    message.hidden = false;
  }

  element("btn-telegram").addEventListener("click", () => {
    dir.x = dir.y = dir.rot = 0;
    dialog.showModal();
    text.focus();
  });
  element("telegram-schliessen").addEventListener("click", () =>
    dialog.close(),
  );
  dialog.addEventListener("click", (event) => {
    if (event.target === dialog) dialog.close();
  });
  file.addEventListener("change", async () => {
    const selected = file.files[0];
    if (!selected) return;
    if (selected.size > 16384) {
      showMessage("Datei ist zu groß", true);
      file.value = "";
      return;
    }
    try {
      const content = await selected.text();
      if (content.length > 4096) {
        showMessage("Nachricht ist zu lang (höchstens 4096 Zeichen)", true);
        file.value = "";
        return;
      }
      text.value = content;
      markdown.checked = true;
      message.hidden = true;
    } catch {
      showMessage("Datei konnte nicht gelesen werden", true);
      file.value = "";
    }
  });
  element("telegram-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if (!text.value.trim()) {
      showMessage("Nachricht ist leer", true);
      return;
    }
    send.disabled = true;
    showMessage("Wird gesendet …");
    try {
      const response = await fetch("/telegram/send", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ text: text.value, markdown: markdown.checked }),
      });
      const result = await response.json();
      if (!response.ok) {
        showMessage(result.fehler || "Senden fehlgeschlagen", true);
        return;
      }
      showMessage("Nachricht gesendet");
      text.value = "";
      file.value = "";
    } catch {
      showMessage("Roboter nicht erreichbar", true);
    } finally {
      send.disabled = false;
    }
  });
})();
