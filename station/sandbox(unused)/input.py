import hid


for device in hid.enumerate():

    print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")


gamepad = hid.device()
gamepad.open(0x057e, 0x2009)
gamepad.set_nonblocking(True)

while True:
    report = gamepad.read(64)
    if report:
        print(report)