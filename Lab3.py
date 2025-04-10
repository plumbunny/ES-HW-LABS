from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEException, UUID

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        super().__init__()

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered:", dev.addr)

# Step 1: Scan
print("🔍 Scanning for 10 seconds...")
scanner = Scanner().withDelegate(ScanDelegate())

try:
    devices = scanner.scan(10.0)
except BTLEException as e:
    print("❌ Scan failed:", e)
    exit(1)

# Step 2: Show devices
devices_list = list(devices)
for i, dev in enumerate(devices_list):
    print(f"[{i}] {dev.addr} (RSSI: {dev.rssi} dB)")
    for (adtype, desc, value) in dev.getScanData():
        print(f"    {desc}: {value}")

# Step 3: Choose device
try:
    index = int(input("Enter device number to connect: "))
    selected = devices_list[index]
except (IndexError, ValueError):
    print("❌ Invalid selection.")
    exit(1)

# Step 4: Connect
print(f"🔗 Connecting to {selected.addr}...")
try:
    dev = Peripheral(selected.addr, selected.addrType)
    print("✅ Connected.")
except BTLEException as e:
    print("❌ Failed to connect:", e)
    exit(1)

# Step 5: Find Notify/Indicate Characteristics
try:
    for svc in dev.getServices():
        for ch in svc.getCharacteristics():
            props = ch.propertiesToString()
            if "INDICATE" in props or "NOTIFY" in props:
                print(f"🧬 Found characteristic: {ch.uuid} ({props})")
                descriptors = ch.getDescriptors(forUUID=0x2902)
                if descriptors:
                    cccd = descriptors[0]

                    # Smart CCCD value selection
                    if "INDICATE" in props:
                        cccd_value = b"\x02\x00"
                    elif "NOTIFY" in props:
                        cccd_value = b"\x01\x00"
                    else:
                        print("❌ Characteristic doesn't support Notify or Indicate.")
                        dev.disconnect()
                        exit(1)

                    print(f"✍️ Writing {cccd_value.hex()} to CCCD {cccd.uuid}")
                    dev.writeCharacteristic(cccd.handle, cccd_value, withResponse=True)
                    print("✅ CCCD set to", cccd_value.hex())
                   
                else:
                    print("⚠️ No CCCD descriptor found")

except BTLEException as e:
    print("❌ Error during CCCD write:", e)
try:
    testService = dev.getServiceByUUID(UUID(0xfff0))
    for ch in testService.getCharacteristics():
        print(str(ch))
    ch=dev.getCharacteristics(uuid=UUID(0xfff1))[0]
    if(ch.supportsRead()):
        print(ch.read())
except BTLEException as e:
    print("❌ Error during CCCD write:", e)

dev.disconnect()
print("🔌 Disconnected.")

