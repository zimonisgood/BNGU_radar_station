# 点云预处理

`field_filter_node` 订阅 `/livox/lidar`（`sensor_msgs/msg/PointCloud2`），
删除场地矩形以外及 XYZ 含 NaN/Inf 的点，发布 `/radar/points_filtered`。
要求驱动使用 `xfer_format=0` 或其他 FLOAT32 XYZ 的 PointCloud2；不接收 Livox CustomMsg。
输入输出使用 SensorDataQoS（best effort，depth=5）。

## 场地与安装姿态

编辑 `config/field_filter.yaml`，参数均在启动时读取，修改后重启节点。
默认长宽 10×10 米、旋转角全零仅为占位，必须根据现场测量修改。

- 场地对齐坐标系原点与雷达光学原点重合，X 沿场地长度，Y 沿宽度，Z 向上，遵守右手定则。
- 保留范围为 `[min_x_m, min_x_m + length_m] × [min_y_m, min_y_m + width_m]`，包含边界。
- 默认 X 为 0～10 米、Y 为 -5～5 米，即雷达处于短边中点。雷达也可以在场外，只需填写真实边界坐标。
- 雷达高于地面不影响水平矩形裁剪：地面 Z 可为负值。本节点不限制 Z，不删除场内地面点。
- 用 `p_field = Rz(yaw) * Ry(pitch) * Rx(roll) * p_lidar` 计算裁剪坐标。
  参数单位为度，正方向按右手定则，依次作用 roll、pitch、yaw。
  雷达绕 X 轴倒置且 X 轴向场地俯倾 15°，在该约定下可用 `(180, 15, 0)`；必须核实实际轴向。
- 旋转仅用于裁剪判断。输出 XYZ、frame_id、时间戳和其余字段保留输入值，点云压平为 height=1。
  不发布 TF，也不把输出伪装为场地坐标系；在 RViz 中可先选 `livox_frame` 为 Fixed Frame。
- 如果驱动或上游已变换过点云，参数必须对应实际输入坐标，不能重复补偿。

固定雷达无需 IMU 即可完成此功能。安装姿态可以实测或通过地面平面拟合校准；
地面法向只能约束横滚/俯仰，场地朝向和边界还需要测量。
如果支架或平台会运动，静态 YAML 不再充分，需要有时间同步和外参标定的动态姿态来源，
可能包括外置 IMU。当前节点不实现 IMU 融合或动态 TF。

## 编译与运行

在 `ros2_jazzy` 容器中：

```bash
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select pointcloud_preprocessing
source install/setup.bash
ros2 launch pointcloud_preprocessing field_filter.launch.py
```

另一个容器终端启动雷达驱动：

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch livox_ros2_driver livox_lidar_launch.py
```

可通过 `config:=/absolute/path/field_filter.yaml` 指定其他配置。
测试：`colcon test --packages-select pointcloud_preprocessing`，然后
`colcon test-result --verbose`。测试覆盖边界、非有限点、安装旋转、字段保留、
大小端、带行填充的有序点云、空输出和错误输入。
