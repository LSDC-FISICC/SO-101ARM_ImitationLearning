import mujoco
import mujoco.viewer
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from so101_mujoco_utils import set_initial_pose, send_position_command

# Variables del SO101-ARM
LEADER_PORT = "COM4"        # Siempre la estructura es puerto -> ID
LEADER_ID = "so101_leader"   
CONTROL_HZ = 60             # frecuencia de control del leader y de la simulación

def read_leader(leader):
    action = leader.get_action()
    return {k.removesuffix(".pos"): v for k, v in action.items()}

m = mujoco.MjModel.from_xml_path('model/scene.xml')
d = mujoco.MjData(m)

USE_FAKE_LEADER = False   # Eso ya no funciona pero se dejo para no cambiar codigo viejo

if USE_FAKE_LEADER:
    from fake_leader import FakeLeader
    leader = FakeLeader()
else:
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    leader = SO101Leader(SO101LeaderConfig(port=LEADER_PORT, id=LEADER_ID, use_degrees=True))

leader.connect()

try:
    target = read_leader(leader)
    print(target)
    set_initial_pose(d, target)

    steps_per_tick = max(1, int(1.0 / CONTROL_HZ / m.opt.timestep))

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            target = read_leader(leader)          # una lectura por tick
            send_position_command(d, target)      # el comando va a d.ctrl
            for _ in range(steps_per_tick): 
                mujoco.mj_step(m, d)
            viewer.sync()
finally:
    leader.disconnect()