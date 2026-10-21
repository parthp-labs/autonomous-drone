#!/usr/bin/env python3

from pymavlink import mavutil
import time

# Update this after identifying your Raspberry Pi UART device.
SERIAL_PORT = "/dev/ttyAMA3"

# Common telemetry baud rate; must match the Pixhawk configuration.
BAUD_RATE = 57600


def main():
    print(f"Connecting to Pixhawk on {SERIAL_PORT}...")
    print(f"Baud rate: {BAUD_RATE}")

    try:
        connection = mavutil.mavlink_connection(
            SERIAL_PORT,
            baud=BAUD_RATE
        )

        print("Serial port opened.")
        print("Waiting for Pixhawk heartbeat...")

        heartbeat = connection.wait_heartbeat(timeout=30)

        if heartbeat is None:
            print("No heartbeat received within 30 seconds.")
            print("Check UART configuration, wiring, and baud rate.")
            connection.close()
            return

        print("\nPixhawk connected!")
        print(f"System ID: {connection.target_system}")
        print(f"Component ID: {connection.target_component}")
        print("Receiving MAVLink messages...\n")

        while True:
            message = connection.recv_match(
                blocking=True,
                timeout=2
            )

            if message is None:
                print("Waiting for telemetry...")
                continue

            message_type = message.get_type()

            if message_type == "BAD_DATA":
                continue

            print(f"{message_type}: {message.to_dict()}")

    except KeyboardInterrupt:
        print("\nReceiver stopped.")

    except Exception as error:
        print(f"Error: {error}")


if __name__ == "__main__":
    main()
