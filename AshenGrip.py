# Hi everyone, this is the code used to simulate keyboard input with the received serial input
# if you haven't seen my video showcasing my custom dark souls controllers
# make sure to check it at exampleurl.com

import serial # we use this library for reading the serial input
import time
import keyboard  # I used this library to get input from the real keyboard
from pynput.keyboard import Controller  # this one is to simulate keyboard input

kb = Controller()  # Initialize the keyboard controller

held_keys = set()  # Track held keys

try:
    ser = serial.Serial(port='COM2', baudrate=9600, timeout=5)  # Add timeout to prevent hanging
    time.sleep(0.1)

    print("Serial port status:", ser.is_open)
    print("Press ESC to exit...")

    while True:  # Keep reading until Esc is pressed
        if ser.in_waiting:  # Check if data is available
            data = ser.readline().decode('utf-8').strip()  # Read and decode the data
            print("Received:", data)

            if len(data) == 1:  # If a single letter is received, press and release it
                kb.press(data)
                time.sleep(0.04)  # Small delay so presses register
                kb.release(data)
            elif len(data) >= 2:  # If more than one letter is received
                first_char, second_char = data[0], data[1]

                if first_char == 'P':  # If first letter is 'P', hold the second letter
                    if second_char not in held_keys:
                        kb.press(second_char)
                        held_keys.add(second_char)  # Track held key
                elif first_char == 'R':  # If first letter is 'R', release the second letter
                    if second_char in held_keys:
                        kb.release(second_char)
                        held_keys.remove(second_char)  # Remove from tracked keys

        if keyboard.is_pressed("esc"):  # Check if Esc key is pressed using the keyboard module
            ser.close()
            print("ESC key pressed. Exiting...")
            break

except serial.SerialException as e:
    print(f"Error: {e}")
except PermissionError:
    print("Permission denied: Another program might be using COM38. Close it and try again.")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    for key in held_keys:  # Ensure all held keys are released on exit
        kb.release(key)
    print("All held keys released.")
