import time

from lerobot.robots.motor_test import RobstrideTest, RobstrideTestConfig

config = RobstrideTestConfig(
    port="can0",  # ou "slcan0" / autre suivant ton setup
    motor_config={"joint_1": (0x01, 0x01, "ELO5")},
    cameras={},
)

robot = RobstrideTest(config)

robot.connect(calibrate=True)

for _ in range(500):
    tgt = 30
    action = {"joint_1.pos": tgt}
    robot.send_normalized_action(action)
    obs = robot.get_observation()
    print(obs)
    time.sleep(0.01)

robot.disconnect()
