# pybulletX Patch

![Package Version](https://img.shields.io/badge/version-0.4.2-orange)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/zhangzrjerry/pybulletxpatch/python-package.yml)
[![Downloads](https://pepy.tech/badge/pybulletx)](https://pepy.tech/project/pybulletx)

A lightweight PyBullet wrapper for robotics researchers. Build robot simulations with less code and reduced boilerplate. Supports Python 3.8 through 3.12.

## Installation

The preferred way of installation is through pip:

```bash
pip install pybulletX@git+https://github.com/ZhangzrJerry/PybulletxPatch.git
```

Alternatively, you can clone the repository and install the package using:

```bash
git clone https://github.com/ZhangzrJerry/PybulletxPatch.git
cd pybulletX/ && pip install -e .
```

## Examples

Here is an example of controlling Kuka arm with PyBulletX.

```python
import time

import numpy as np
import pybullet as p
import pybulletX as px

P_GAIN = 50
desired_joint_positions = np.array([1.218, 0.507, -0.187, 1.235, 0.999, 1.279, 0])

def main():
    px.init()

    robot = px.Robot("kuka_iiwa/model.urdf", use_fixed_base=True)
    robot.torque_control = True

    while True:
        time.sleep(0.01)

        error = desired_joint_positions - robot.get_states().joint_position
        actions = robot.action_space.new()
        actions.joint_torque = error * P_GAIN
        robot.set_actions(actions)

        p.stepSimulation()

if __name__ == "__main__":
    main()
```

Here is the same example but without PyBulletX.

```python
import time

import numpy as np
import pybullet as p
import pybullet_data

P_GAIN = 50
desired_joint_positions = np.array([1.218, 0.507, -0.187, 1.235, 0.999, 1.279, 0])

def main():
    p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    num_dofs = 7
    joint_indices = range(num_dofs)

    # The magic that enables torque control
    p.setJointMotorControlArray(
        bodyIndex=robot_id,
        jointIndices=joint_indices,
        controlMode=p.VELOCITY_CONTROL,
        forces=np.zeros(num_dofs),
    )

    while True:
        time.sleep(0.01)

        joint_states = p.getJointStates(robot_id, joint_indices)
        joint_positions = np.array([j[0] for j in joint_states])
        error = desired_joint_positions - joint_positions
        torque = error * P_GAIN

        p.setJointMotorControlArray(
            bodyIndex=robot_id,
            jointIndices=joint_indices,
            controlMode=p.TORQUE_CONTROL,
            forces=torque,
        )

        p.stepSimulation()

if __name__ == "__main__":
    main()
```

The examples above are available in `examples/with_pybulletX.py` and `examples/without_pybulletX.py`.

## License

PyBulletX is licensed under [MIT License](LICENSE).
