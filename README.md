# Battle Bots (FSE 100 Osburn)

This is a redo of the given `client_control.py` from professor Osburn's FSE 100
class to work with linux.

## How it works

Instead of using pynput which does not capture linux inputs well, so decided
to change the program to use evdev.

To start we cycle through the connected device to discover any keyboards connected
and return it.

```python
#client_control.py
for path in list_devices():
        dev = InputDevice(path)
        if "keyboard" in dev.name.lower() or "kbd" in dev.name.lower():
            return dev
```
