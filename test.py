import sys
sys.path.append('/home/pi/.local/lib/python3.7/site-packages')
sys.path.append('/home/pi/Team4121/Libraries')
sys.path.append('/usr/local/lib/vmxpi/')
from FRCNavxLibrary import FRCNavx
from time import sleep
gyro = FRCNavx("Navx")
while True:
    print("yaw: {}. pitch: {}".format(gyro.read_yaw(), gyro.read_pitch()))
    sleep(0.1)
