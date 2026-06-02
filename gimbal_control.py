import os
import time
from dynamixel_sdk import *

# Initialize port and packet handler
portHandler = PortHandler('/dev/cu.usbmodem101')
packetHandler = PacketHandler(2.0)

# Open port
if portHandler.openPort() and portHandler.setBaudRate(57600):
    print("Succeeded to open the port")

# Motor ID and control table address for XL-330
DXL_1 = 1
DXL_2 = 2
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116

#### Velocity Control ####

# # Disable torque
# packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 0)
# packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 0)
# # Enable velocity control
# packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
# packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
# # Enable torque
# packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 1)
# packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 1)

# # Spin continuously
# # A value of 200 is roughly 50% speed. Use negative values (e.g., -200) to reverse direction.
# packetHandler.write4ByteTxRx(portHandler, DXL_1, ADDR_GOAL_VELOCITY, 100)
# packetHandler.write4ByteTxRx(portHandler, DXL_2, ADDR_GOAL_VELOCITY, 100)

# time.sleep(5)

# # # Stop the motor
# packetHandler.write4ByteTxRx(portHandler, DXL_1, ADDR_GOAL_VELOCITY, 0)
# packetHandler.write4ByteTxRx(portHandler, DXL_2, ADDR_GOAL_VELOCITY, 0)


# # Disable Torque and Close Port
# packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 0)
# packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 0)
# portHandler.closePort()

#### Position Control ###

if portHandler.openPort() and portHandler.setBaudRate(57600):
    print("Reopened port for position control test")
    
    # Angle command
    angle = 0

    # Write goal position
    goal_position = 1600

    # Motor 1
    packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
    packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write4ByteTxRx(portHandler, DXL_1, ADDR_GOAL_POSITION, goal_position)

    time.sleep(3)

    # Motor 2
    packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
    packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write4ByteTxRx(portHandler, DXL_2, ADDR_GOAL_POSITION, goal_position)

    time.sleep(3)

    # Disable torque and close port
    packetHandler.write1ByteTxRx(portHandler, DXL_1, ADDR_TORQUE_ENABLE, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_2, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()

    portHandler.closePort()