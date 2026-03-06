# BVH 数据格式转换链路完整分析

本文以 **BVH 文件** 为例，从磁盘文件到机器人运动 pkl 的整条数据转换链路做逐段说明。

---

## 一、总览：格式转换链路图

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│  BVH 文本文件 (.bvh)                                                                      │
│  [HIERARCHY 骨架 + MOTION 帧数据：欧拉角 + 根位移，单位 cm]                                │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼ read_bvh()
                                        │  general_motion_retargeting/utils/lafan_vendor/extract.py
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│  Anim 对象 (内存)                                                                         │
│  quats: (T, J, 4) 局部四元数 wxyz；pos: (T, J, 3) 局部/根位置；parents, bones(names)       │
│  注：frametime 被解析但未存入 Anim，不随 Anim 返回                                         │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼ load_bvh_file()
                                        │  general_motion_retargeting/utils/lafan1.py
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│  统一人体帧序列 lafan1_data_frames + actual_human_height                                  │
│  每帧 = dict: body_name -> [position_3d, quaternion_wxyz]，世界系，单位 m，+ Left/RightFootMod │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼ GMR.retarget()  [内部调用 update_targets()]
                                        │  general_motion_retargeting/motion_retarget.py
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│  MuJoCo qpos (每帧)                                                                       │
│  [root_pos(3), root_quat(4) wxyz, joint_angles(nv-7)]，与 model 的 qpos 顺序一致          │
└─────────────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼ bvh_to_robot.py 保存逻辑
                                        │  scripts/bvh_to_robot.py
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│  机器人运动 pkl (motion_data)                                                             │
│  fps, root_pos (N,3), root_rot (N,4) xyzw, dof_pos (N, nv-7),                           │
│  local_body_pos (None in bvh_to_robot.py), link_body_list (None in bvh_to_robot.py)     │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

下面按阶段展开每一层的具体格式与代码位置。

---

## 二、阶段 1：BVH 文件 → Anim（read_bvh）

**代码位置**：`general_motion_retargeting/utils/lafan_vendor/extract.py` — `read_bvh(filename, start, end, order)`。

### 2.1 BVH 文件格式（输入）

- **HIERARCHY**：递归定义骨架
  - `ROOT name`、`JOINT name`：关节名存入 `names`（即后来的 `bones`）
  - `OFFSET x y z`：关节相对父关节的偏移（单位一般为 cm）
  - `CHANNELS n ...`：
    - 根节点多为 6：先 3 个位置 (X Y Z)，再 3 个旋转（如 Xrotation Yrotation Zrotation）
    - 子关节多为 3：仅旋转
  - 旋转通道名到轴的映射：`Xrotation→x, Yrotation→y, Zrotation→z`，解析出的顺序记为 `order`（如 `"zyx"`）
- **MOTION**：
  - `Frames: N`
  - `Frame Time: dt`（注：`dt` 被解析到局部变量 `frametime`，但**未存入 Anim 对象也未返回**；`bvh_to_robot.py` 通过 `--motion_fps` 参数手动指定帧率，默认 30）
  - 每行一帧：一串浮点数 = root 的 3 位置 + 各关节的 3 个欧拉角（度），或根据 CHANNELS 拆成多段

### 2.2 解析结果：Anim

- **quats**：`(T, J, 4)`，局部旋转，**四元数 wxyz**；由 `utils.euler_to_quat(np.radians(rotations), order=order)` 得到（`utils.py:56`），并做 `remove_quat_discontinuities`（`utils.py:251`）。
- **pos**：`(T, J, 3)`。初始化为所有关节的 OFFSET 值（`positions = offsets[np.newaxis].repeat(fnum, axis=0)`）；
  - channels=3（最常见）：仅根节点（index 0）从帧数据更新位置，子关节保留 OFFSET；
  - channels=6：所有关节从帧数据更新位置。
- **offsets**：各关节 OFFSET（骨架 T-pose 中相对父关节的偏移，cm）。
- **parents**：父关节下标。
- **bones**：与 `names` 一致，即 BVH 里 ROOT/JOINT 的名字（如 "Hips", "Spine", "LeftUpLeg", ...）。

要点：**此阶段已完成「欧拉角 → 局部四元数」和「BVH 通道 → 统一 Anim 结构」**，尚未做坐标系或单位变换。帧率信息在此阶段丢失。

---

## 三、阶段 2：Anim → 统一人体帧序列（load_bvh_file）

**代码位置**：`general_motion_retargeting/utils/lafan1.py` — `load_bvh_file(bvh_file, format="lafan1")`。

### 3.1 前向运动学与坐标系

1. **FK**：`utils.quat_fk(data.quats, data.pos, data.parents)`（`utils.py:88`）
   - 输入：局部四元数、局部位置、父索引
   - 输出：每帧每关节的**世界系**四元数 `global_data[0]` 和世界系位置 `global_data[1]`（单位仍与 BVH 一致，一般为 cm）。

2. **坐标系与单位**（与 LAFAN1 等约定一致）：
   - 旋转：左乘一个固定旋转矩阵对应的四元数
     `rotation_matrix = [[1,0,0], [0,0,-1], [0,1,0]]`
     即 Y-up → Z-up 坐标系对齐。
   - 位置：`position = global_data[1][frame, i] @ rotation_matrix.T / 100`
     先同坐标系变换，再 **cm → m**。

### 3.2 每帧的字典结构（GMR 通用人体格式）

对每一帧构造一个字典 `result`：

- **键**：骨骼名 `bone`（来自 `data.bones`）。
- **值**：`[position, orientation]`
  - `position`：3D 向量，世界系，米。
  - `orientation`：四元数，**wxyz**，世界系。

在此基础上按 `format` 增加足部修正关节（用于 IK 映射）：

- **lafan1**：`LeftFootMod` / `RightFootMod` = 脚位置 + 脚趾朝向（用 LeftToe/RightToe 的朝向）。
- **nokov**：用 `LeftToeBase` / `RightToeBase` 的朝向。

返回值：`(frames, human_height)`，其中 `frames` 是上述字典的列表，`human_height` 当前代码里**固定为 1.75**（注释掉了从数据自动计算的逻辑）。

**重要**：从这里开始，所有后续模块使用的「人体」格式都是：
**dict[body_name] = [np.array(3,), np.array(4,)]（位置 + wxyz 四元数）**。
IK 配置中的 `human_root_name`、`human_scale_table`、`ik_match_table*` 里用到的都是这些 body_name（如 "Hips", "Spine2", "LeftFootMod" 等）。

---

## 四、阶段 3：人体帧 → 机器人 qpos（retarget）

**代码位置**：`general_motion_retargeting/motion_retarget.py` — `update_targets()` + `retarget()`。

**IK 配置查找**：`general_motion_retargeting/params.py` — `IK_CONFIG_DICT[src_human][tgt_robot]`。
  - `src_human` 由 `bvh_to_robot.py` 传入：`f"bvh_{args.format}"`，即 `"bvh_lafan1"` 或 `"bvh_nokov"`（不是 `"bvh"`）。
  - 对应 JSON 配置在 `general_motion_retargeting/ik_configs/` 目录下，如 `bvh_lafan1_to_g1.json`。

### 4.1 配置与人体→机器人映射

- IK 配置（如 `bvh_lafan1_to_g1.json`）指定：
  - `human_root_name`（如 "Hips"）
  - `human_height_assumption`：配置假设的人体身高，与 `actual_human_height` 之比作为整体缩放因子 `ratio`
  - `human_scale_table`：各人体关节的缩放系数（会再乘以 `ratio`）
  - `ik_match_table1` / `ik_match_table2`：
    `robot_frame_name -> [human_body_name, pos_weight, rot_weight, pos_offset, rot_offset]`
    即「机器人 body 帧 — 人体 body」的对应关系及权重、偏移。

### 4.2 update_targets(human_data)

对当前帧的 `human_data`（即上一步的某一个 `result` 字典），**依次执行**：

1. **to_numpy**：保证每个 body 的 pos/rot 为 numpy 数组。
2. **scale_human_data**：按 `human_scale_table` 和根节点把各关节位置缩放（根直接乘系数，其它在根坐标系下缩放再还原到世界系）；旋转不变。
3. **offset_human_data**：仅应用 **table1 的偏移**（`pos_offsets1`, `rot_offsets1`）——先旋转偏移再位置偏移（位置偏移在更新后的旋转局部系下计算），注意 **table2 的偏移（`pos_offsets2`, `rot_offsets2`）在此步骤中不被应用**。
4. **apply_ground_offset**：整体沿 z 下移 `ground_offset`（默认 0，可由 `set_ground_offset()` 设置）。
5. 可选 **offset_human_data_to_ground**：把最低足部压到地面高度（`offset_to_ground=True` 时生效）。

得到 `self.scaled_human_data`（供可视化使用）。
然后对 `ik_match_table1` / `ik_match_table2` 中出现的每个 `human_body_name`，用 `scaled_human_data` 中的 `pos` 和 `quat` 设置对应 mink **FrameTask** 的 target（SE3）。

### 4.3 retarget()

- 用 **mink** 在 MuJoCo 的 `Configuration` 上解两阶段 IK（若 `use_ik_match_table1/2` 为 True）：
  - 先解 table1 的任务，再解 table2 的任务；
  - 每阶段使用**收敛循环**（while 检查误差减少量 > 0.001，最多 `max_iter=10` 次），而非单次求解；
  - 求解器：`solver="daqp"`，阻尼：`damping=5e-1`。
- 返回 **configuration.data.qpos.copy()**，即 MuJoCo 的**完整 qpos**：
  - 前 3：根位置（m）
  - 接着 4：根四元数，**wxyz**（MuJoCo 约定）
  - 其余：各关节角（rad），顺序与 `model` 的 DoF 顺序一致。

---

## 五、阶段 4：qpos → 机器人运动 pkl（保存与回放）

**代码位置**：
- 单文件入口 `scripts/bvh_to_robot.py`（保存逻辑在尾部）
- 批量入口 `scripts/bvh_to_robot_dataset.py`（额外用 `KinematicsModel` 计算 `local_body_pos`）
- 读取：`general_motion_retargeting/data_loader.py` — `load_robot_motion()`

### 5.1 每帧 qpos 的拆分

- `qpos[:3]` → **root_pos**
- `qpos[3:7]` → 根四元数；保存时从 **wxyz 转为 xyzw**：`qpos[3:7][[1,2,3,0]]`，得到 **root_rot**
- `qpos[7:]` → **dof_pos**（所有关节角）

### 5.2 写入 pkl 的 motion_data

```python
motion_data = {
    "fps": motion_fps,           # 来自 --motion_fps 参数（默认 30），非从 BVH 文件读取
    "root_pos": root_pos,        # (N, 3)
    "root_rot": root_rot,        # (N, 4) xyzw
    "dof_pos": dof_pos,          # (N, nv-7)
    "local_body_pos": None,      # bvh_to_robot.py 中为 None；bvh_to_robot_dataset.py 中由 KinematicsModel 计算
    "link_body_list": None,      # 同上
}
pickle.dump(motion_data, f)
```

### 5.3 回放时的读取

`load_robot_motion()`（`general_motion_retargeting/data_loader.py`）会：

- 把 `root_rot` 从 **xyzw 转回 wxyz**：`motion_data["root_rot"][:, [3,0,1,2]]`
- 与 `root_pos`、`dof_pos` 一起交给 `RobotMotionViewer.step(...)` 做可视化或控制（`general_motion_retargeting/robot_motion_viewer.py`）。

---

## 六、各阶段数据格式对照表

| 阶段           | 位置/变量              | 位置单位 | 旋转表示     | 根四元数存储 | 关键字段/结构 |
|----------------|------------------------|----------|--------------|--------------|-------------------------------|
| BVH 文件       | 磁盘 .bvh              | cm       | 欧拉角(度)   | -            | HIERARCHY + MOTION 行        |
| Anim           | extract.read_bvh 返回  | cm       | 局部四元数   | wxyz         | quats(T,J,4), pos(T,J,3), bones |
| 人体帧         | lafan1_data_frames[i]  | m        | 世界四元数   | wxyz         | dict[body_name]=[pos, quat]   |
| 缩放后人体     | scaled_human_data      | m        | 世界四元数   | wxyz         | 同上，用于 set_target 和可视化 |
| 机器人 qpos    | configuration.data.qpos| m, rad   | 根四元数+关节角 | wxyz (MuJoCo) | [root_pos(3), root_quat(4), dof...] |
| 运动 pkl       | motion_data            | m, rad   | 根四元数     | **xyzw**     | root_pos, root_rot, dof_pos  |
| 回放加载       | load_robot_motion      | m, rad   | 根四元数     | 转回 wxyz    | 供 Viewer.step 使用          |

---

## 七、涉及代码文件汇总

| 文件路径 | 作用 |
|----------|------|
| `scripts/bvh_to_robot.py` | 单文件 BVH 转机器人运动的主入口；含保存逻辑 |
| `scripts/bvh_to_robot_dataset.py` | 批量处理 BVH 文件；额外用 `KinematicsModel` 计算 `local_body_pos` |
| `general_motion_retargeting/utils/lafan_vendor/extract.py` | `read_bvh()`：BVH 文本 → Anim 对象 |
| `general_motion_retargeting/utils/lafan_vendor/utils.py` | `quat_fk()`、`euler_to_quat()`、`remove_quat_discontinuities()` 等数学工具 |
| `general_motion_retargeting/utils/lafan1.py` | `load_bvh_file()`：Anim → 统一人体帧序列 |
| `general_motion_retargeting/motion_retarget.py` | `GeneralMotionRetargeting`：`update_targets()` + `retarget()` + 缩放/偏移辅助函数 |
| `general_motion_retargeting/params.py` | `ROBOT_XML_DICT`、`IK_CONFIG_DICT`：机器人模型路径与 IK 配置路径映射 |
| `general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json` | 示例 IK 配置（lafan1 → G1）；同目录下有其他机器人/格式的配置 |
| `general_motion_retargeting/data_loader.py` | `load_robot_motion()`：从 pkl 加载并做 xyzw→wxyz 转换 |
| `general_motion_retargeting/robot_motion_viewer.py` | `RobotMotionViewer`：MuJoCo 可视化与视频录制 |
| `general_motion_retargeting/kinematics_model.py` | `KinematicsModel`：正向运动学，批量数据集处理时用于计算 `local_body_pos` |

---

## 八、BVH 特有约定小结（lafan1 / nokov）

- **骨骼名**：由 BVH HIERARCHY 决定，必须与 IK 配置里的 `human_scale_table`、`ik_match_table*` 的 body 名一致（如 Hips, Spine2, LeftUpLeg, LeftFootMod, RightFootMod 等）。
- **足部**：lafan1 使用 LeftToe/RightToe 生成 LeftFootMod/RightFootMod；nokov 使用 LeftToeBase/RightToeBase，对应不同的 BVH 骨骼命名。
- **格式参数**：`load_bvh_file(..., format="lafan1"|"nokov")` 与 `src_human="bvh_lafan1"|"bvh_nokov"` 需一致，以在 `IK_CONFIG_DICT`（`params.py`）中选对 IK 配置。
- **帧率**：BVH 文件中的 `Frame Time` 字段被 `read_bvh()` 解析但**丢弃**（未存入 Anim）；`bvh_to_robot.py` 通过 `--motion_fps` 参数（默认 30）手动指定，保存的 pkl 中 `fps` 字段使用该值。

以上即从 **BVH 文件 → Anim → 统一人体帧 → 机器人 qpos → 运动 pkl** 的完整数据格式转换链路；以 BVH 为例，其它输入格式（如 SMPL-X、FBX）会在一开始换成各自的加载器，但进入「统一人体帧」之后的后半段与本文一致。
