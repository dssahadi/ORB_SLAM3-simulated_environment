# ORB_SLAM3-simulated_environment

1. Segue os passos da primeira pasta para clonar e buildar o ORB_SLAM3.

```bash
cd Dev/ORB_SLAM3/Thirdparty/Sophus/build
sudo make install
```

2. Segue os passos da segunda pasta para clonar e buildar o ambiente da simulação.

3. Mova as pastas dentro de src/ para a pasta src/ dentro da frtl_2025_ws.

```bash
colcon build
```
4. Para testar: Run Task -> simulate -> fase1_25. Depois: Run Task -> agent.

Em um terminal:

```bash
ros2 run ros_gz_image image_bridge /camera/image_raw /camera/depth/image_raw
```

Em outro terminal:

```bash
ros2 run orb_slam3_ros2_wrapper rgbd   ~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt   ~/frtl_2025_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/gazebo_rgbd.yaml   --ros-args   -r /depth/image_raw:=/camera/depth/image_raw   -p use_sim_time:=true
```

Por fim:

```bash
ros2 launch cbr_fase1 simulation.launch.py
``` 
