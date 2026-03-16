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
ros2 run ros_gz_bridge parameter_bridge /world/fase1_25/model/x500_simulation_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU --ros-args -r /world/fase1_25/model/x500_simulation_0/link/base_link/sensor/imu_sensor/imu:=/imu0
```

Para rodar o ORB_SLAM3:

```bash
ros2 run orb_slam3_ros2_wrapper mono_imu \
  ~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/frtl_2025_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/gazebo_mono_imu.yaml \
  --ros-args \
  -p use_sim_time:=true \
  -r imu:=/imu0
```

Por fim:

```bash
ros2 launch cbr_fase1 simulation.launch.py
``` 

OBS: foi testado no dia 16/03/2026 e funcionou, entretanto cabe ressaltar que por algum motivo o ORB_SLAM3 ficava forçando seu fechamento, além disso a angulação da câmera dentro da simulação não é favorável aos testes e talvez seja necessário utilizar outra fase para melhor teste.
