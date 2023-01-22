from machine import Pin, ADC
import time, array, sys, os, json


#     Hobbyist Steno Keyboard - MicroPython driver for a hobbyist steno machine (keyboard)
#     Copyright (C) 2023 Thomas TEMPE
# 
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <https://www.gnu.org/licenses/>.

#TODO:
# Send the console messages as strokes interpretable by Plover
# Add support for more protocols: Plover-HID, rollover keyboard
# Give calibration success/failure feedback to the user (through LED color codes?)
# Add a way of displaying the current protocol, and the current Bluetooth pairing partner
# Add 1st-hand-up support
# Add key repeat support
# Add a way to send parameters or commands from a Plover plug-in
# Measure power consumption. See if we can decrease it by toggling the Hall effect sensors on and off

#Keymap
#address  0  1  2  3  4  5  6  7  | 8  9  10  11  12  13  14  15 | 16  17  18  19  20  21  22  23 | 24 25 26 27 28 29 30 31
#Key      S- S- #  T- K- -  P- W- | A  H- R-  O   *   *   E   F- | -R  -U  -P  -B  -L  -G  -   -T | -S #  -D -Z

#index = address
__key_names = ["S1", "S2", "#", "T", "K", "-", "P", "W", "A", "H", "R", "O", "*1", "*2", "E", "F", "-R", "-U", "-P", "-B", "-L", "-G", "--", "-T", "-S", "#2", "-D", "-Z"]
__finger_spelling = {"a":["A", "#", "-P"]}
__gemini_codes = {"S":0x006000000000, "T":0x001000000000, "K":0x000800000000, "P":0x000400000000, "W":0x000200000000,
                  "H":0x000100000000, "R":0x000040000000, "A":0x000020000000, "O":0x000010000000, "#":0x3F000000007E,
                  "*":0x00000C300000, "E":0x000000080000, "U":0x000000040000,"-P":0x000000004000}

### Extract from Plover code:
# In the Gemini PR protocol, each packet consists of exactly six bytes
# and the most significant bit (MSB) of every byte is used exclusively
# to indicate whether that byte is the first byte of the packet
# (MSB=1) or one of the remaining five bytes of the packet (MSB=0). As
# such, there are really only seven bits of steno data in each packet
# byte. This is why the STENO_KEY_CHART below is visually presented as
# six rows of seven elements instead of six rows of eight elements.

#__protocol = [["N/A", "Num", "Num", | "Num", "Num", "Num", "Num"],
#              ["S-",  "S-",  "T-",  | "K-",  "P-",  "W-",  "H-" ],
#              ["R-",  "A",   "O-",  | "*",   "*",   "N/A", "N/A"],
#              ["N/A", "*",   "*",   | "E",   "U",   "-F",  "-R" ],
#              ["-P",  "-B",  "-L",  | "-G",  "-T",  "-S",  "-D" ],
#              ["Num", "Num", "Num", | "Num", "Num", "Num", "-Z" ]]


#Here is the same protocole, but with key addresses (see keymap above) instead
__protocol = [[ 28,  2,  2,  2,  2,  2,  2],
              [  0,  1,  3,  4,  6,  7,  9],
              [ 10,  8, 11, 12, 13, 28, 28],
              [ 28, 12, 13, 14, 17, 15, 16],
              [ 18, 19, 20, 21, 23, 24, 26],
              [  2,  2,  2,  2,  2,  2, 27]]

__hw_cal_file = "hardware_calibration.json"

class Keys: 
    def __init__(self):
        #Pins
        self.muxA = Pin( 9, Pin.OUT)
        self.muxB = Pin(10, Pin.OUT)
        self.muxC = Pin(11, Pin.OUT)
        self.muxInh = Pin(12, Pin.OUT)
        self.ADC = [ ADC(Pin(i, Pin.IN)) for i in [26, 27, 28, 29]]
        self.LED_act = Pin(25, Pin.OUT)
        self.LED_cal = Pin(24, Pin.OUT)
        self.SW_cal = Pin(19, Pin.IN, Pin.PULL_UP)
        
        #Buffers
        self.readings = array.array("I", (0 for i in range(32)))    #16-bit ADC reading
        self.output = array.array("B", (0 for i in range(32)))      # 8-bit normalized values
        self.prev_output = array.array("B", (0 for i in range(32))) # same
        self.serial = sys.stdout.buffer
        self.gemini_buffer = bytearray(6)

        #bitmap of currently pressed keys
        self.pressed = 0
        self.prev_pressed = 0
        self.stroke = 0
        
        #hardware calibration values (used for converting 16-bit readings to 8-bit values)
        try:
            fd = open(__hw_cal_file, "r")
            (self.zero, self.max) = json.load(fd)
            fd.close()
        except OSError:
            self.zero = [32913]*32 
            self.max =  [51227]*32
            
        #User calibration: thresholds for converting 8-bit values to pressed/release values (with hysteresis), between 0 (fully released) and 255 (fully pressed)
        self.thresh_h = [128]*32  
        self.thresh_l = [100]*32
        #bitmap of the addressable sensors that are actually used in the steno machine (1st mux, 1st sensor is 0b1)
        self.mask = 0b1101101111111111111111011111 
        self.l_mask = 0b111111111111                 #left hand
        self.r_mask = 0b1111111111111100000000000000 #right hand
        
    def set_address(self, mux):
            self.muxInh(1)
            time.sleep_ms(1)
            self.muxA(mux&1)
            self.muxB(mux&2)
            self.muxC(mux&4)
            self.muxInh(0)

    def read(self):
        "One loop of polling all the keys. Get the results in the object attributes. Returns whether there was a change in the 8-bit normalized readings"
        (self.prev_output, self.output) = (self.output, self.prev_output)
        changed = False
        self.prev_pressed = self.pressed
        #Read the 16-bit analog values and normalize them
        for i, mux in enumerate(range(8)):
            self.set_address(mux)
            for j, ADC in enumerate(self.ADC):
                addr=i+j*8
                if (self.mask >> addr) & 1:
                    #Read analog
                    self.readings[addr] = val = ADC.read_u16()
                    #Normalize
                    val = abs(val-self.zero[addr])*255//abs(self.max[addr]-self.zero[addr])
                    self.output[addr] = val = min(255, max(0, val))
                    changed += (val != self.prev_output[addr])
                    #Threshold (with hysteresis)
                    if val > self.thresh_h[addr] and not((self.pressed>>addr)&1):
                        self.pressed |= 1<<addr
                    elif val < self.thresh_l[addr] and (self.pressed>>addr)&1:
                        self.pressed -= 1<<addr
        return self.pressed != self.prev_pressed

    def write(self):
        "Write out the current stroke on the console using Gemini PR protocol"
        self.LED_act(1)
        for c in range(6):
            self.gemini_buffer[c] = 0
            for i, j in enumerate(__protocol[c]):
                self.gemini_buffer[c] += ((self.stroke>>__protocol[c][i])&0x01)<<(6-i)
        self.gemini_buffer[0] |= 0x80
        sys.stdout.buffer.write(self.gemini_buffer)
        time.sleep_ms(100)
        self.LED_act(0)

    def calibrate(self):
        "Hardware calibration sequence"
        
        #Phase 1: hold the calibration button for at least 2 seconds. Read the "resting" key values.
        t0 = time.ticks_ms()
        elapsed = lambda :time.ticks_ms()-t0
        vMin0 = [65535]*32
        vMax0 = [0]*32
        time.sleep_ms(100)#anti-rebound
        print("Starting calibration.\nPlease hold the calibration button until flashing stops. Please don't press the keys during that time.")
        while(not(self.SW_cal())):
            self.LED_cal((time.ticks_ms()>>6) & 1 and (elapsed() < 2000)) #Flash rapidly for the first 2 seconds, then turn off
            self.read()
            #There is some jitter. Measure both the hightest and lowest value, and choose the best one at the end of phase 2
            vMin0 = [min(vMin0[i], self.readings[i]) for i in range(32) ]
            vMax0 = [max(vMax0[i], self.readings[i]) for i in range(32) ]
        if elapsed() < 2000:
            self.LED_cal(0)
            print("Calibration aborted.")
            return
        
        #Phase 2: record the max (or min) values
        vPressed = vMin0[:]
        print("Please now press down each key all the way\nPress the calibration when you are done")
        while (self.SW_cal()):
            self.LED_cal((time.ticks_ms()>>8)&1) #Flash slowly
            self.read()
            #Depending on the magnet orientation, values are either increasing or decreasing relative to the resting position. Take the most distant one
            vPressed = [self.readings[i] if abs(self.readings[i]-vMin0[i])>abs(vPressed[i]-vMin0[i])  else vPressed[i] for i in range(32) ]
        #Select the best resting position depending on the magnet orientation
        zero = [vMin0[i] if vPressed[i]<vMin0[i] else vMax0[i] for i in range(32)]
        self.LED_cal(1)
        
        #Control that the calibration is good
        #TODO after hardware is stabilized: check that all keys in self.mask have been pressed, otherwise roll back with an error message
        err = []
        for i in range(32):
            if self.mask&(1<<i) and abs(vPressed[i] - zero[i]) < 2:
                err.append(__key_names[i])
        if len(err):
            print("Error: the following keys were not calibrated: ", err)
            return
        
        #Record calibration to file, and apply it
        self.max = vPressed
        self.zero = zero
        fd = open(__hw_cal_file, "w")
        json.dump((self.zero, self.max), fd)
        fd.close()
        self.LED_cal(0)
        print("calibration recorded. Using the new values")
        time.sleep_ms(100) #anti-rebound
        while(not(self.SW_cal())):
            #Waiting for key release
            pass
        time.sleep_ms(100) #anti-rebound

    def loop(self):
        "Main loop for using as a steno machine, using the Gemini PR protocole"
        if not(self.SW_cal()):
            #Calibration button was held down on startup. Revert to factory values.
            print("Reverting calibration to factory defaults")
            try:
                os.remove(__hw_cal_file)
            except OSError:
                pass
            self.__init__()
            #Wait for cal button release
            while not(self.SW_cal()):
                pass
            
        while True:
            #Read keyboard
            if self.read():
                #something changed
                if self.pressed == 0:
                    #All keys released
                    self.write()
                    self.stroke = 0
                else:
                    self.stroke |= self.pressed
                    
            #See if we need to start a calibration sequence
            if not(self.SW_cal()):
                self.calibrate()


def monitor_readings():
    "Real-time display of the 32 readings (raw sensor data), for display with Thonny's View->Plotter feature"
    while True:
        k.read()
        print(k.readings)
        time.sleep_ms(100) 

def monitor_normalized():
    "Real-time display of all 25 keys' 8-bit normalized readings"
    listed_keys = []
    for i in range(32):
        if (k.mask >>i)&1:
            listed_keys.append(i)
    while True:
        k.read()
        print([k.output[i] for i in listed_keys])
        time.sleep_ms(100)

def monitor_output():
    "Real-time display of the keys' on-off status"
    while True:
        k.read()
        if k.pressed != k.prev_pressed:
            t = bin(k.pressed)[2:]
            print(("0"*(27-len(t)))+t)
            time.sleep_ms(10)
    

def minmax():
    """Loop read the keyboard, and display the 32 highest and lowest values identified so far.
    Use it to gauge the level of noise (don't move any keys), or for calibration (press down each key fully)
    """
    min_val = array.array("I", (65535 for i in range(32)))
    max_val = array.array("I", (0 for i in range(32)))
    last_time = 0
    while True:
        k.read()
        for i, v in enumerate(k.readings):
            max_val[i]=max(v, max_val[i])
            min_val[i]=min(v, min_val[i])
        if (time.ticks_ms() - last_time > 1000):
            last_time = time.ticks_ms()
            print("Min: ", min_val)
            print("Max: ", max_val)
            time.sleep_ms(50)
                    
def read_one(mux, input):
    "Gives the measured voltage for one sensor"
    k.set_address(input)
    print("S{}-{} : {:.3} V".format(mux, input, k.ADC[mux].read_u16()*3.3/65535)) 

def time_once():
    "How many microseconds to poll once?"
    t=time.ticks_us()
    k.read()
    print("Polling once takes", time.ticks_us()-t, "us")

k = Keys()
#monitor_readings()
k.loop()