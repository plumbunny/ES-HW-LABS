from bluepy.btle import Peripheral, UUID, Scanner, DefaultDelegate
import struct
import threading
import queue

TARGET_MAC = "fe:46:66:11:4b:99"

# UUIDs
RPi_CMD_UUID = UUID("00000003-0001-11e1-ac36-0002a5d5c51b")
ACCEL_CHAR_UUID = UUID("00e00000-0001-11e1-ac36-0002a5d5c51b")
SERVICE_UUID = UUID("00000000-0001-11e1-9ab4-0002a5d5c51b")

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        super().__init__()

class NotificationDelegate(DefaultDelegate):
    def __init__(self, accel_char):
        super().__init__()
        self.accel_char = accel_char

    def handleNotification(self, cHandle, data):
        if cHandle == self.accel_char.getHandle():
            t, x, y, z = struct.unpack('<Hhhh', data)
            print(f" t: {t}, x: {x}, y: {y}, z: {z}")

def input_thread(freq_queue):
    while True:
        user_input = input("⏱️ Enter new sampling frequency (Hz): ").strip()
        if user_input.isdigit():
            freq_queue.put(int(user_input))
        else:
            print("lease enter a valid integer.")

def main():
    print("Scanning for devices...")
    scanner = Scanner().withDelegate(ScanDelegate())
    devices = scanner.scan(5.0)

    target = None
    for dev in devices:
        if dev.addr == TARGET_MAC:
            target = dev
            print(f" Found device {dev.addr} (RSSI={dev.rssi})")
            break

    if not target:
        print("Target device not found.")
        return

    print("Connecting...")
    dev = Peripheral(TARGET_MAC, "random")

    print("Discovering services...")
    service = dev.getServiceByUUID(SERVICE_UUID)

    print("Dumping all characteristics:")
    for char in dev.getCharacteristics():
        print(f"- {char.uuid} (handle: 0x{char.getHandle():04X})")

    accel_char = dev.getCharacteristics(uuid=ACCEL_CHAR_UUID)[0]
    rpi_cmd_char = dev.getCharacteristics(uuid=RPi_CMD_UUID)[0]

    # Enable notifications
    cccd = accel_char.getDescriptors(UUID(0x2902))[0]
    cccd.write(b"\x01\x00", withResponse=True)
    print("Notifications enabled")

    dev.setDelegate(NotificationDelegate(accel_char))

    freq_queue = queue.Queue()
    threading.Thread(target=input_thread, args=(freq_queue,), daemon=True).start()

    print("Listening for notifications and user input...")

    try:
        while True:
            if dev.waitForNotifications(1.0):
                continue

            while not freq_queue.empty():
                freq_val = freq_queue.get()
                print(f"Sending frequency update: {freq_val} Hz")

                # Format: [command_id=0x00, freq_LSB, freq_MSB]
                freq_payload = bytearray([0x00, freq_val & 0xFF, (freq_val >> 8) & 0xFF])
                rpi_cmd_char.write(freq_payload, withResponse=False)

                print(f"Sent sampling frequency = {freq_val} Hz")

    except KeyboardInterrupt:
        print("Disconnecting...")
        dev.disconnect()

if __name__ == "__main__":
    main()
