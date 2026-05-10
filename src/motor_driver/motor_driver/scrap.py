# import math
# inches_to_meters = 0.0254
# wheel_circumference = 13*math.pi*inches_to_meters
# print(wheel_circumference)
from roboclaw_python.roboclaw_3 import Roboclaw
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
address = 0x80
roboclaw.Open()
roboclaw.ReadEncM1(address)
roboclaw.ReadEncM2(address)