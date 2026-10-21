#!/usr/bin/env python3
import socket
from evdev import InputDevice, ecodes, list_devices

def get_keyboard():
    """Auto-detects the active keyboard input device or prompts for fallback."""
    for path in list_devices():
        dev = InputDevice(path)
        if "keyboard" in dev.name.lower() or "kbd" in dev.name.lower():
            return dev
    
    # Fallback to listing devices if auto-detect fails
    print("Could not auto-detect keyboard. Available devices:")
    for path in list_devices():
        dev = InputDevice(path)
        print(f"  {dev.path}: {dev.name}")
    path = input("\nEnter keyboard device path (e.g., /dev/input/event4): ")
    return InputDevice(path)

def control_ev3(ev3_ip, ev3_port):
    """
    Connects to an EV3 server and controls it using keyboard inputs via evdev.
    
    Args:
        ev3_ip (str): IP address of the EV3 server.
        ev3_port (int): Port number of the EV3 server.
    """
    try:
        keyboard = get_keyboard()
    except Exception as e:
        print(f"Error initializing keyboard device: {e}")
        return

    # Map numeric key codes to command characters
    key_map = {
        ecodes.KEY_W: 'w',  # Forward
        ecodes.KEY_A: 'a',  # Left
        ecodes.KEY_S: 's',  # Backward
        ecodes.KEY_D: 'd',  # Right
        ecodes.KEY_F: 'f',  # Weapon
        ecodes.KEY_Q: 'q',  # Quit
    }

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        print(f"Connecting to EV3 at {ev3_ip}:{ev3_port}...")
        client_socket.connect((ev3_ip, ev3_port))
        print("Connected successfully!")

        print(f"\nListening on {keyboard.name} ({keyboard.path})")
        print("Press 'q' to quit\n")

        # Exclusive input grab
        try:
            keyboard.grab()
        except OSError:
            print("Warning: Could not grab device exclusively. Continuing without grab...")

        for event in keyboard.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code not in key_map:
                    continue

                key_char = key_map[event.code]

                # event.value: 1 = press, 0 = release (2 = hold, ignored)
                if event.value == 1:  # Key pressed
                    command = f"{key_char}d".encode()  # 'd' for down
                    print(f"Sending: {command.decode()}")
                    client_socket.sendall(command)

                elif event.value == 0:  # Key released
                    command = f"{key_char}u".encode()  # 'u' for up
                    print(f"Sending: {command.decode()}")
                    client_socket.sendall(command)

                # Stop if 'q' is pressed
                if key_char == 'q' and event.value == 1:
                    print("\nQuit key pressed, exiting...")
                    break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Ungrab keyboard cleanly
        try:
            keyboard.ungrab()
            print("Keyboard released")
        except Exception:
            pass

        # Close socket connection
        client_socket.close()
        print("Socket closed")


if __name__ == "__main__":
    EV3_IP = "192.168.0.204"  # Replace with your EV3's IP address
    EV3_PORT = 12345

    control_ev3(EV3_IP, EV3_PORT)
