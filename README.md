# quadcopter-with-mojuco

# Bitcraze Crazyflie 2 Description (MJCF)

Requires MuJoCo 2.2.2 or later.

## Overview

This package contains a simplified robot description (MJCF) of the Crazyflie 2
model from [Bitcraze](https://www.bitcraze.io/). It is derived from the publicly
available [ROS description](https://github.com/whoenig/crazyflie_ros).

<p float="left">
  <img width="1846" height="814" alt="screenshot" src="https://github.com/user-attachments/assets/d2112a9f-ca9f-4551-aeee-7474891c1e8b" />
</p>

## URDF → MJCF Conversion

1. Converted the DAE mesh file in `crazyflie_description` to OBJ format using [Blender](https://www.blender.org/).
2. Processed the OBJ file with [obj2mjcf](https://github.com/kevinzakka/obj2mjcf).
3. Added a `<freejoint>` to the root body, and some lighting in the XML file.
4. Set the inertial properties to values obtained from the datasheet and [MIT's system identification](https://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf)
    * These properties are set via inertial tag i.e. `pos ="0 0 0" mass="0.027" diaginertia="2.3951e-5 2.3951e-5 3.2347e-5"`
5. Added combined thrust and body moments about a `site` placed at the inertial frame. The `ctrlrange` limits are currently arbitrary and need to be further tuned.
6. Added `scene.xml` which includes the quadrotor, with a textured ground plane and skybox.


## demo show
![10](https://github.com/user-attachments/assets/7d6274c1-784a-46f1-9914-e42e822d1164)

![20](https://github.com/user-attachments/assets/8d87fe5b-44ff-4bd0-9355-59f8a28f2fe3)

![30](https://github.com/user-attachments/assets/37fe8c80-7fa1-42b4-ba00-e673d84ed32d)

![40](https://github.com/user-attachments/assets/1e5d4226-aed3-4e2d-a20c-355d2325b8f4)

![50](https://github.com/user-attachments/assets/1f283c74-04c8-4525-b22b-7f39db863ed4)


## License

This model is released under an [MIT License](LICENSE).












