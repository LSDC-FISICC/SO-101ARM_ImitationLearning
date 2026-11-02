import time
import mujoco
import mujoco.viewer
from so101_mujoco_utils import hold_position, move_to_pose, set_initial_pose, send_position_command

m = mujoco.MjModel.from_xml_path('model/scene.xml')
d = mujoco.MjData(m)

starting_position = {
    'shoulder_pan': 0.0, 'shoulder_lift': -92.25, 'elbow_flex': 88.80,
    'wrist_flex': 61.88, 'wrist_roll': -91.67, 'gripper': 0.0,
}

set_initial_pose(d, starting_position)

sequence = [
    ({'shoulder_pan': 0.0,   'shoulder_lift': 0.0,   'elbow_flex': 0.0,
      'wrist_flex': 0.0,     'wrist_roll': 0.0,      'gripper': 0.0},   2.0, 2.0),   # brazo recto pues q=[0,0,0,0,0,0]
    ({'shoulder_pan': 40.0,  'shoulder_lift': -20.0, 'elbow_flex': 30.0,
      'wrist_flex': 20.0,    'wrist_roll': 0.0,      'gripper': 30.0},  2.0, 1.0),
    ({'shoulder_pan': -40.0, 'shoulder_lift': 20.0,  'elbow_flex': -20.0,
      'wrist_flex': -20.0,   'wrist_roll': 45.0,     'gripper': 10.0},  2.0, 1.0),
    (starting_position,                                                 2.0, 1.0),   # ponerse acostado el brazo
]

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():          
        for pose, move_time, hold_time in sequence:
            if not viewer.is_running(): 
                break
            move_to_pose(m, d, viewer, pose, move_time)
            hold_position(m, d, viewer, hold_time)