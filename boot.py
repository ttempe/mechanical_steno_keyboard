#Make the flash writable from the microcontroller code, to save calibration data
import storage
storage.remount('/', readonly=False)
#Enable two USB serial ports, one for console and one for Plover
import usb_cdc
usb_cdc.enable(console=True, data=True)
