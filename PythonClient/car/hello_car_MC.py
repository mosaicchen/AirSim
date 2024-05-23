import setup_path
import airsim
import cv2
import numpy as np
import os
import time
import tempfile

# connect to the AirSim simulator
client = airsim.CarClient(ip="192.168.1.137")
client.confirmConnection()
client.enableApiControl(True)
print("API Control enabled: %s" % client.isApiControlEnabled())

mcMsg = airsim.CarMCMsg()
mcMsg.msg="This Message is from VM-Ubuntu-Airsim"
client.setCarMCMsg(mcMsg)

while(1):
    recvMsg = client.getCarMCMsg()
    print(recvMsg.msg)
    time.sleep(0.1)   # let car drive a bit


#restore to original state
# client.reset()

# client.enableApiControl(False)
