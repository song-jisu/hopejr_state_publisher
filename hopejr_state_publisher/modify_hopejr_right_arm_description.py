import pathlib
import os

path = pathlib.Path(os.path.join(
    os.path.dirname(__file__),
    '../hopejr_right_arm_description/launch/view_robot.launch.py'
)).resolve()

text = path.read_text()
text = text.replace("joint_state_publisher_gui", "hopejr_state_publisher_gui")
path.write_text(text)