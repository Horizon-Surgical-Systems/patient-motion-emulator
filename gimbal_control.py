import os
import time
from dynamixel_sdk import *

# Initialize port and packet handler
portHandler = PortHandler('/dev/cu.usbmodem1101')
packetHandler = PacketHandler(2.0)

# Open port
if portHandler.openPort() and portHandler.setBaudRate(57600):
    print("Succeeded to open the port")

# Motor ID and control table address for XL-330
DXL_1 = 1
DXL_2 = 2
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 0

# Disable torque
packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 0)
# Enable velocity control
packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_OPERATING_MODE, 1)
packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_OPERATING_MODE, 1)
# Enable torque
packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 1)
packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 1)

# Spin continuously
# A value of 200 is roughly 50% speed. Use negative values (e.g., -200) to reverse direction.
packetHandler.write4ByteTxRx(portHandler, DXL_1, ADDR_GOAL_VELOCITY, 100)
packetHandler.write4ByteTxRx(portHandler, DXL_2, ADDR_GOAL_VELOCITY, 100)


time.sleep(5)

# Stop the motor
packetHandler.write4ByteTxRx(portHandler, DXL_1, ADDR_GOAL_VELOCITY, 0)
packetHandler.write4ByteTxRx(portHandler, DXL_2, ADDR_GOAL_VELOCITY, 0)


# Disable Torque and Close Port
packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 0)
portHandler.closePort()

# Write goal position
# goal_position = 1023
# packetHandler.write4ByteTxRx(portHandler, DXL_1, ADDR_GOAL_POSITION, goal_position)
# packetHandler.write4ByteTxRx(portHandler, DXL_2, ADDR_GOAL_POSITION, goal_position)
# portHandler.closePort()