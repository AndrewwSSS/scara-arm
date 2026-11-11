# Browser console

[`tools/console.html`](../tools/console.html) is a serial console for the jog
console, with configurable command buttons. It talks to the board over
[Web Serial](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API).

## Browser support

Chrome, Edge and recent Firefox-based browsers support Web Serial. Safari does
not.

## Connecting

1. Close `tools/monitor.py` and the PlatformIO monitor: a serial port has only
   one owner at a time.
2. Open the page and press **Connect**, then pick the board's port.
3. Press **Disconnect** before uploading firmware, so the upload can use the port.

An Uno restarts each time the port opens.

## Reconnecting

The page remembers the last board and reconnects to it automatically, both when
the page opens and when the board is plugged back in. Pressing **Disconnect**
turns this off until the next **Connect**.

- **Connect** opens the remembered board without asking.
- **Other port…** picks a different one.

Web Serial identifies a port only by its USB vendor and product ID. If two
plugged-in boards use the same USB chip, the page does not guess between them;
use **Other port…**.

How long the browser remembers the permission depends on the browser:

- **Firefox-based browsers** forget the port when the page reloads, so the first
  **Connect** after a reload asks again.
- **Chrome and Edge** keep it, but only for a page served over HTTP, not one
  opened as a file. To serve it locally:

  ```sh
  python3 -m http.server --bind 127.0.0.1 --directory tools 8000
  ```

  Then open <http://localhost:8000/console.html>.

## Buttons

Buttons are stored in the browser, separately for each address: the file page
and `localhost` each have their own set.

- **Edit buttons** adds, changes and removes them.
- **Export** / **Import** move them between browsers or addresses as a JSON file.
  [`tools/jog-console-buttons.json`](../tools/jog-console-buttons.json) is a
  ready-made set to import.
- A command containing `{n}` asks for the value when pressed, for example
  `g {n}`.
