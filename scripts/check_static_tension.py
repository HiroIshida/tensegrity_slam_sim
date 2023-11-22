import mujoco
import mujoco_viewer
import numpy as np
from rospkg import RosPack
import time

# MJCFファイルのパス
rospack = RosPack()
model_path = rospack.get_path('tensegrity_slam_sim') + '/models/scene_real_model.xml'  
#model_path = rospack.get_path('tensegrity_slam_sim') + '/models/scene_real_model_fullactuator.xml'  

tension_threshold = -50  # 張力のしきい値
current_tension = 0  # アクチュエータの初期張力

# モデルとシミュレーションのロード
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

data.qpos += np.array([0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0.5, 0, 0, 0, 0,
                0, 0, 0.5, 0, 0, 0, 0
                ])

# Viewerの初期化
viewer = mujoco_viewer.MujocoViewer(model, data)

class PIDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.desired_tendon_length_min = 0.28
        self.desired_tendon_length_max = 0.29
        self.reset()
    
    def reset(self):
        self.previous_error = 0
    
    def step(self, tendon_length):
        error = tendon_length - self.desired_tendon_length_max
        derivative = (error - self.previous_error)
        output = self.kp * error + self.kd * derivative
        self.previous_error = error
        output_clip = np.clip(output, model.actuator_ctrlrange[i, 0], model.actuator_ctrlrange[i, 1])
        return output_clip


class ConstantController:
    def __init__(self, tension):
        self.tension = tension

    def step(self, tendon_length):
        return self.tension

use_constant = True
debug_print = False
if use_constant:
    controller = ConstantController(-15.0)
else:
    controller = PIDController(20.0, 2)

while True:
    print(data.sensordata)
    for i in range(model.nu):
        a = data.sensordata[i] 
        b = data.ten_length[i]
        data.ctrl[i] = controller.step(a)
        if debug_print and np.abs(a - b) > 1e-3:
            print("error at ", i, "th actuator")
            print("sensordata: ", a)
            print("ten_length: ", b)
    mujoco.mj_step(model, data)
    viewer.render()
    print("ctrl: ", data.ctrl)
