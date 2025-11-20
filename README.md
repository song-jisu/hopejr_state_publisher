# hopejr_state_publisher
Visualize with RViz, but use motor value, not joint value.

## Usage
Create a new workspace and clone this library:
```shell
mkdir hopejr_right_arm
cd hopejr_right_arm
mkdir src
cd src
git clone https://github.com/song-jisu/hopejr_arm_description.git
git clone https://github.com/song-jisu/hopejr_hand_description.git
git clone https://github.com/song-jisu/hopejr_right_arm_description.git
git clone https://github.com/song-jisu/hopejr_state_publisher.git
```

Modify `view_robot.launch.py`
```shell
cd src/hopejr_state_publisher/hopejr_state_publisher
python3 modify_hopejr_right_arm_description.py
```

Build:
```shell
cd hopejr_right_arm
colcon build --symlink-install
. install/setup.bash
```

Run:
```shell
ros2 run hopejr_right_arm_description view_robot.launch.py
```

<img width="944" height="574" alt="image" src="https://github.com/user-attachments/assets/e2035124-95c3-465c-8a3e-d783f1d35057" />
