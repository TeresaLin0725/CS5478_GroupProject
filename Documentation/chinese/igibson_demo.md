# 真实地图与实体机器人演示（2026-10-14）

当前主入口为[完整 A/B/C/W3 真实 LLM 管线录像](../../runs/integrated-llm-final-20261018/recording/index.html)，详见[完整管线与验收记录](integrated_pipeline.md)。下文记录历史独立演示。

历史入口：[交互回放](../../runs/igibson-realistic-final-20261014/index-legacy-before-integration-20261017.html)。页面可以切换难度和策略、暂停、拖动时间、跳转决策事件。画面直接来自 iGibson 渲染器，第三人称中显示实体 Locobot，旁边同步显示机器人 RGB 视角、RGB-D 地图和扫描平面图。

2026-10-17：W3 已合并，默认动态场景 Nearest 已重录为 15 fps 连续视频，包含扫描和停车过程，相机改为保持当前侧并平滑换位；其他历史条件明确标为稀疏回放。详见 [W3 合并、流畅度与双向反馈检查](w3_merge_and_demo.md)。

## 项目已有资源

本次没有重新造房屋。复用已下载的 Gibson `Rs` 公寓扫描：`integration/data/igibson-data/g_dataset/Rs/mesh_z_up.obj`、扫描贴图和平面图。机器人使用 `robot_assets/assets/models/locobot/locobot.urdf`，目标使用公开 YCB `003_cracker_box`。运行环境为 `integration/data/venv-igibson`。资源来自 [Stanford iGibson](https://stanfordvl.github.io/iGibson/scenes.html)。

三个难度是同一真实扫描公寓的配置，不是三套独立房屋：

| 配置 | 难度来源 |
|---|---|
| apartment | 客厅、卧室、卫生间、厨房；门洞、扫描家具和台面目标遮挡；初始看不到目标 |
| clutter | 新增软凳、客厅椅子和卧室椅子；路线需避开家具及安全膨胀范围 |
| dynamic | 软凳和卧室椅子，另有带轮推车；推车按时间连续横穿通道，再退回；地图只能通过观测更新 |

新增家具通过 URDF 导入 iGibson，包含椅腿、坐垫、靠背、推车轮子和层板的独立碰撞形状。尺寸、位置和推车运动在 `simulation/indoor_challenges.py` 中固定，便于复现。Locobot 从起点初始化后只接受轮速命令，搜索过程不传送机器人。

## 运行

在项目根目录：

```powershell
cd integration
$env:PYTHONPATH = "$PWD/src"
./data/venv-igibson/Scripts/python.exe -m frontier_search.runner.igibson_demo --scenario suite --policy all --backend replay
```

当前 PowerShell 执行策略禁止 `.ps1` 时，使用上面的 Python 命令。系统允许执行脚本时，也可以从项目根目录使用便捷入口：

```powershell
./integration/scripts/run_igibson_demo.ps1 -Scenario suite -Policy all -Backend replay
./integration/scripts/run_igibson_demo.ps1 -Scenario dynamic -Policy nearest
./integration/scripts/run_igibson_demo.ps1 -Scenario clutter -Policy information_gain
./integration/scripts/run_igibson_demo.ps1 -Scenario dynamic -Policy llm_guided -Backend live
```

默认离线。每次使用新的 `runs/igibson-realistic-时间戳` 目录，保留 JSONL、摘要、WebP 逐帧回放、GIF 和首末帧。suite 逐个启动独立进程，避免多个仿真器共享一个 OpenGL 上下文。环境不存在时再执行 `python integration/scripts/setup_igibson.py`，本机已有资源不必重下载。

LLM 离线条件从同一场景 Information Gain 运行中保存 frontier ID，再通过相同 LLM 接口回放；页面和日志均标明不是模型推理。实时模式读取本地 `.env`，密钥不写入产物。四个策略在观测到目标后都优先接近目标。

## 近距离完成和日志

目标的实例分割需满足像素数要求，位置从渲染深度估计。两帧远距离可见只建立目标 track，不结束搜索。规划从已观测空闲区域寻找目标接近点；需要两次不同仿真时间的停稳观测，估计平面距离不超过 0.85 米。独立验收读取真实目标位置，核对两帧和当前机器人均在 0.93 米内，并核对停稳与定位误差。接近台面时，渲染观察视角会根据已观测目标的位置朝上调整，避免台面物体移出低位相机的视野；这是演示的观察视角控制。

日志记录决策输入、候选可达性、选择、实际行驶距离、重规划、执行失败和独立验收。逐帧 trace 的时间、位姿与距离均来自实际仿真过程。页面不会把基础方块动画当作实测场景。

RGB-D 融合和规划采用演示专用循环。家具类别仍是经过渲染深度可见性检查的手工地标，目标来自实例分割；此版本不声称完成共享 W2 管线或 YOLO 的生产验收。Rs 是静态扫描，扫描家具不可移动；新增推车是脚本驱动的运动障碍。底层物理仍使用 iGibson 自带的 PyBullet 引擎，已移除的是独立基础 PyBullet 演示。

旧运行记录归档到 `runs/archive-pybullet/`。共享矩形车身规划、转身扫掠、目标接近优先级和恢复回归测试保留。

## 本地实测

三种难度 × 四种策略共 12 个 episode 均完成近距离目标验收，全部零障碍接触。默认离线 LLM 回放与同场景 IG 的记录一致。

| Nearest 场景 | 行驶距离 | 重规划 | 导航失败后改选 | 最终距目标 |
|---|---:|---:|---:|---:|
| 原始公寓 | 14.87 m | 1 | 0 | 0.78 m |
| 拥挤布局 | 10.86 m | 1 | 0 | 0.79 m |
| 动态推车（2026-10-17 重录） | 10.42 m | 1 | 0 | 0.79 m |

[产物完整性检查](../../runs/igibson-realistic-final-20261014/verification.json) 检查逐帧文件、真实位姿/距离、初始目标隐藏、终止验收、回放来源和日志校验和。
[页面运行检查](../../runs/igibson-realistic-final-20261014/viewer-check.json) 检查难度/策略切换、时间拖动、暂停、重播和事件跳转。当前工具没有可用浏览器，因此页面控制使用离线 DOM 检查；已抽查各场景渲染画面。

[完整回归日志](../../runs/igibson-realistic-final-20261014/tests.log)：2026-10-17 合并后 350 项测试通过，0 失败、0 跳过。
