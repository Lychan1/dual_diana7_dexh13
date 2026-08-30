# DexH13 灵巧手 URDF 末端安装与坐标校正需求

## 1. 背景

当前模型的目标灵巧手就是 DexH13。现有 URDF 已包含 DexH13 法兰、手掌和手指模型，但末端固定连接的坐标原点、姿态、mesh 偏移或左右手命名可能存在问题，导致手掌与法兰安装面对不准、末端姿态错误或左右手镜像关系错误。本需求用于校正现有 DexH13 模型，不是替换成另一款灵巧手。

当前右臂末端链为：

```text
right_arm_link_7 -> right_sensor_link -> right_dexh13_flange -> right_palm_link
```

当前左臂末端链为：

```text
left_arm_link_7 -> left_sensor_link -> left_dexh13_flange -> left_palm
```

相关定义位于 `diana7_dual_dexh13_description/urdf/dual_diana7_dexh13.urdf`。法兰和手掌之间使用 fixed joint，姿态由该 joint 的 `origin xyz/rpy` 决定；法兰 mesh 自身也存在 visual/collision origin 偏移。

## 2. 目标

1. 保留 DexH13 作为目标灵巧手，校正其现有 URDF 末端链和固定变换。
2. 保留 Diana7 腕部和传感器的运动学定义不变，除非验证发现腕部定义本身错误。
3. 使 DexH13 手掌安装面与法兰在位置、方向和左右手镜像关系上正确对齐。
4. 统一左右手的 link/joint 命名和 parent-child 关系，避免同一模型使用 `right_palm_link` 与 `left_palm` 造成维护和生成问题。
5. 同步保证 URDF、RViz 和 MJCF 使用一致的末端坐标定义。
6. 保证 DexH13 的质量、质心、惯性、visual mesh 和 collision mesh 参数有效。

## 3. 范围

### 包含

- DexH13 base/palm 及手指 link、joint 的 URDF/Xacro 定义。
- 法兰到 DexH13 手掌基座的固定安装 joint。
- 必要时增加 `adapter_link` 表示转接板或安装座。
- DexH13 visual/collision mesh 路径、缩放和坐标系修正。
- DexH13 inertial 参数和碰撞参数。
- 左、右手独立的安装变换。
- MJCF 中对应的 body、mesh、site、actuator、sensor 和 contact 更新。

### 不包含

- Diana7 七个旋转关节的重新标定。
- 未经验证的腕部 mesh 或传感器几何重建。
- 手指 tactile taxel 布局的重新设计，除非现有 DexH13 传感器布局确实需要调整。

## 4. 设计要求

### 4.1 坐标系

- 法兰坐标系原点应明确位于安装面基准点，通常为安装面中心。
- 法兰坐标系的法向轴、切向轴和零位方向必须形成右手坐标系。
- DexH13 基座 link 的坐标系应明确记录原点和三个轴的物理含义。
- CAD、URDF、ROS 和 MuJoCo 使用的长度单位必须统一为米；角度统一为弧度。
- 不得通过随意旋转 mesh 来掩盖 joint 坐标系错误。mesh 偏移必须有明确的 CAD 坐标系依据。

### 4.2 末端链结构

推荐结构为：

```text
arm_link_7 -> sensor_link -> flange -> adapter_link (可选) -> dexh13_hand_base
```

法兰和 DexH13 基座之间必须使用独立的 fixed joint，例如：

```xml
<joint name="right_dexh13_mount_joint" type="fixed">
  <parent link="right_dexh13_flange" />
  <child link="right_dexh13_hand_base" />
  <origin xyz="dx dy dz" rpy="roll pitch yaw" />
</joint>
```

DexH13 左右手的 `xyz/rpy` 必须分别测量或从 CAD 装配体导出，不得默认复制右侧参数。

### 4.3 物理参数

- 每个有质量的 link 必须包含 `inertial`。
- 质量、质心和惯性矩应来自正确材料密度和装配状态下的 CAD 质量属性。
- 惯性矩必须相对于 inertial origin 定义，并满足物理有效性和正定性要求。
- DexH13 的电机、减速器、外壳和安装件是否纳入质量模型必须记录在文档或配置中。
- 仅用于坐标标记的 frame/site 不应引入虚假的可观质量。

### 4.4 碰撞与显示

- visual mesh 和 collision mesh 应分开维护。
- collision mesh 应尽量简化，避免手指自碰撞导致仿真不稳定。
- DexH13 安装面、法兰和转接板的碰撞几何不得产生明显重叠或穿透。
- mesh 的 scale 必须与文件实际单位一致，并在 RViz/MuJoCo 中检查尺寸。

## 5. 验收标准

### 运动学验收

- [ ] `check_urdf` 或等效 XML/URDF 解析成功。
- [ ] 从 `arm_link_7` 到 DexH13 手掌基座的 parent-child 链唯一且无断链。
- [ ] DexH13 安装面中心与法兰基准点的位置误差不超过项目规定阈值；默认建议小于 1 mm。
- [ ] 安装面法向和绕法向的姿态误差不超过项目规定阈值；默认建议小于 1 度。
- [ ] 左右手分别在 RViz 中验证，镜像关系和朝向正确。
- [ ] 机械臂关节角变化不会改变法兰与 DexH13 之间的固定安装关系。

### 几何和仿真验收

- [ ] 所有 visual/collision mesh 路径可解析，尺寸和单位正确。
- [ ] DexH13 与法兰、转接板之间无不合理穿透。
- [ ] DexH13 collision 几何不会造成初始状态爆炸或持续接触抖动。
- [ ] URDF 生成的 MJCF 成功加载，或自定义 MJCF 生成脚本成功运行。
- [ ] DexH13 所需 actuator、sensor、site 和 tactile 配置已明确保留或调整。
- [ ] 新增 link 的质量和惯性参数通过基本数值检查。

## 6. 推荐实施步骤

1. 从 DexH13 CAD 装配体确定手掌基座坐标系，并核对现有 visual/collision mesh。
2. 在独立的 Xacro/URDF 文件中整理 DexH13，不直接改写自动生成的 tactile taxel 块。
3. 先只连接 `flange -> dexh13_hand_base`，在 RViz 中用坐标轴和网格校正 `origin xyz/rpy`。
4. 若法兰和手之间有实物转接件，增加 `adapter_link`，将转接件偏移单独建模。
5. 补充 DexH13 各 link 的 inertial 和 collision 参数。
6. 完成右手验证后，独立配置左手的安装变换和 mesh。
7. 更新 MJCF 生成输入或脚本，再执行完整生成和加载检查。

## 7. 验证命令

在仓库根目录执行：

```bash
check_urdf diana7_dual_dexh13_description/urdf/dual_diana7_dexh13.urdf
python3 diana7_dual_dexh13_description/scripts/add_paxini_taxels.py
python3 dexh13_mjcf/build_dual_mjcf.py
```

如果使用 ROS 1，还应在已 source 的 Catkin 环境中启动 `display.launch`，检查两侧安装姿态、网格、TF 和碰撞显示。若使用 MuJoCo，应加载生成的 `dexh13_mjcf/dual_diana7_dexh13.xml` 并检查末端姿态和接触稳定性。

## 8. 变更记录要求

每次修改 DexH13 或其安装变换时，应记录：

- DexH13 型号和 CAD/mesh 来源。
- 法兰、DexH13 基座和转接板坐标系定义。
- 左右手的 `origin xyz/rpy`。
- mesh 文件单位、scale 和 collision 简化方式。
- 质量、质心和惯性参数来源。
- 执行过的验证命令及结果。
