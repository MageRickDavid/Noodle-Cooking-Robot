import minimalmodbus
import time
# Configure Modbus instrument
instrument = minimalmodbus.Instrument("/dev/ttyUSB4", 1, mode=minimalmodbus.MODE_RTU)
instrument.serial.baudrate = 9600
instrument.serial.bytesize = 8
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 1  # Timeout in seconds
startADDR = 1  # Starting address for relay control
relay_states = [False] * 8  # Store the states of all relays
def toggle_multiple_relays():
    global relay_states
    while True:
        try:
            relay_input = input("Enter relay numbers (comma-separated, e.g., 1,3,5) or '-1' to exit: ").strip()
            if relay_input == "-1":
                print("Exiting...")
                break
            relay_numbers = [int(num) for num in relay_input.split(",") if num.strip().isdigit()]
            if not relay_numbers:
                print("Invalid input. Please enter numbers between 0 and 7.")
                continue
            state = input("Turn ON or OFF? (on/off): ").strip().lower()
            if state not in ["on", "off"]:
                print("Invalid input. Type 'on' or 'off'.")
                continue
            # Update the relay states in memory
            for relay_num in relay_numbers:
                if 0 <= relay_num <= 7:
                    relay_states[relay_num] = state == "on"
                    instrument.write_bit(startADDR + relay_num, state == "on", functioncode=5)
                    print(f"Relay {relay_num} turned {'ON' if state == 'on' else 'OFF'}.")
                else:
                    print(f"Invalid relay number {relay_num}. Must be between 0 and 7.")
        except ValueError:
            print("Invalid input. Please enter numbers separated by commas.")
def refresh_relays():
    """Continuously refresh relays to prevent auto-reset."""
    while True:
        for i, state in enumerate(relay_states):
            instrument.write_bit(startADDR + i, state, functioncode=5)
        time.sleep(5)  # Refresh every 5 seconds
# Start the relay control and refreshing process
import threading
# refresh_thread = threading.Thread(target=refresh_relays, daemon=True)
# refresh_thread.start()
toggle_multiple_relays()






