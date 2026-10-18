# 新电脑运行与复现

所有命令均在 **Git 仓库根目录**执行，即包含 `pyproject.toml` 的 `integration/` 目录。路径不需要是原作者的 `D:` 盘；使用短的英文路径可减少旧版原生编译工具的路径问题。当前生产入口为 `scripts/run_integrated.py`。

## 1. 运行环境与依赖

实测环境：Windows 11 x64（系统 build 26200）、CPython **3.8.20 x64**、iGibson **2.2.2**、NVIDIA RTX 4060 Laptop GPU。Python 3.8 是仿真器兼容环境；不要用 Python 3.12/3.13 直接安装这组旧版二进制依赖。

| 用途 | 固定版本 / 文件 |
|---|---|
| iGibson 源码 | `7d9598278b95ec1039e1f4366c66b45346521e44`，由安装脚本下载并打 MSVC / 纹理读取补丁 |
| 基础依赖 | `requirements-igibson.txt`：NumPy 1.24.4、SciPy 1.10.1、pybullet-svl 3.1.6.4、OpenCV 4.8.1.78、Pillow 10.4.0 等 |
| GPU 检测 | `requirements-integrated.txt`：torch 2.4.1+cu121、torchvision 0.19.1+cu121、ultralytics 8.3.203 |
| CPU 检测 | `requirements-integrated-cpu.txt`：对应的 +cpu Torch wheel，其他版本相同 |
| 间接依赖 | `constraints-windows-py38.txt` 固定实测环境中的间接依赖；安装器自动应用，CUDA / CPU 的 Torch 分别由上述 requirements 选择 |
| 检测权重 | 官方 `yolo11s.pt`；SHA256 `85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5` |
| 物理场景 | 公开 Gibson Rs 扫描、Locobot 与 YCB `025_mug`；优化版推车使用 YCB `006_mustard_bottle` 的纹理网格，均来自安装脚本下载的资产包 |
| 视频编码 | imageio-ffmpeg 0.5.1 自带 FFmpeg；不要求另外安装系统 FFmpeg |

首次安装需要 Git、[uv](https://docs.astral.sh/uv/getting-started/installation/)、Visual Studio 2022 Build Tools 的“使用 C++ 的桌面开发”（MSVC x64 和 Windows SDK）、可用的显卡驱动和联网。CMake 已包含在固定依赖里。建议准备至少 16 GB 内存和 15 GB 可用磁盘空间，这是项目使用建议，并非测得的最低配置。

本机采用 GPU 版 Torch，其 wheel 自带 CUDA 12.1 运行库；iGibson 的 Windows 构建不启用 CUDA tensor 渲染，不必另外安装完整 CUDA Toolkit。**CPU 选项仅切换 YOLO 推理设备，iGibson 仍需要可工作的原生 OpenGL 显卡渲染**。集成显卡、无显示远程会话和纯软件渲染未在本项目验证。

依赖版本依据本项目实测环境固定。安装背景见 [iGibson 官方说明](https://stanfordvl.github.io/iGibson/installation.html)，Torch wheel 组合见 [PyTorch 官方历史版本](https://pytorch.org/get-started/previous-versions/#v241)。这不是要求升级到它们当前的最新版。

## 2. Windows 首次安装

先取得本次的 `project-source-20261018.zip` 源码交接包，完整解压后进入其中的 `frontier-search/`。交接包包含本地新增的安装脚本、依赖和场景修订；使用 GitHub 分发时，需要先把这些文件提交并同步到所用分支，远端提交 `43617b9` 本身不包含本次新增的安装脚本。

```powershell
cd frontier-search
python scripts/setup_project.py --device cuda
```

`python` 可以是系统已安装的 Python 3；安装器用 uv 单独创建固定 Python 3.8.20 的 `data/venv-igibson/`。安装器依次构建原生仿真器、下载公开场景和机器人、安装检测依赖、安装本项目、下载并校验权重。编译原生扩展需要时间。若系统无法找到 C++ 编译器，可从 VS 的 **x64 Native Tools Command Prompt** 运行同一命令。

YOLO 使用 CPU 的机器改为：

```powershell
python scripts/setup_project.py --device cpu
```

已有依赖与资源的机器可跳过安装。原安装脚本 `setup_igibson.py` 仅安装仿真器；完整管线应使用 `setup_project.py`，否则会缺少 Torch、Ultralytics 或检测权重。不要直接复制其他电脑的虚拟环境：其中的解释器、原生 DLL 与 `.pth` 包含平台和绝对路径信息，应在新电脑重新创建。完整实测包版本见 [环境快照](../runtime-windows-20261018.json)。

环境自检及一次真正的动态场景渲染（不调用任何模型服务）：

```powershell
./data/venv-igibson/Scripts/python.exe scripts/check_environment.py --render --output runs/environment.json
```

检查返回码应为 0，JSON 中 `passed` 应为 `true`，相邻 PNG 是本机生成的场景截图。它检查依赖导入、原生渲染器、场景/机器人/目标文件、权重校验和，以及推车完整扫掠无碰撞与地面支撑。各电脑上的结果保存在各自的 `runs/`，方便排查差异。

## 3. 不需要密钥的完整搜索

```powershell
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-nearest-dryrun
```

这是共享 RGB-D 建图、真实 YOLO、B 导航、C 策略恢复和独立目标验收的完整仿真。默认不录像，结果写入输出目录的 `summary.json` 与 `events.jsonl`。`termination=success` 才表示本次任务完成。其他离线策略为 `information_gain`、`random`，环境可选 `apartment`、`clutter`、`dynamic`。每次使用新的输出目录，防止覆盖运行证据。

启动器会寻找项目内 Windows/Linux 仿真虚拟环境；若使用其他虚拟环境，可以先激活它，或显式指定解释器：

```powershell
$env:FRONTIER_PYTHON = 'C:/your-env/Scripts/python.exe'
python scripts/run_integrated.py --policy nearest --output runs/my-other-env-run
```

从其他工作目录启动也可以，启动器会切换到仓库根目录；相对 `--output` 也按仓库根目录解析。

## 4. 真实 LLM 搜索

```powershell
Copy-Item .env.example .env
# 在编辑器中填写 .env 的 LLM_API_KEY、LLM_MODEL、LLM_BASE_URL
python scripts/run_integrated.py --policy llm_guided --profile dynamic --env-file .env --output runs/my-llm-dryrun
```

使用自己的可用模型与额度。模板中的模型名是原作者本机配置，不保证其他账号可访问。启动器默认优先读仓库根目录 `.env`，兼容原工作区父目录 `.env`；显式 `--env-file` 最清楚。进程中已有的同名环境变量优先于文件。

真实模式向配置的服务发送仿真观察到的类别、虚拟坐标、候选/成本与失败摘要，不发送 RGB、深度图或扫描文件。`.env`、`data/`、`weights/` 和新生成的 `runs/` 默认被 Git 忽略（少量历史演示快照已单独纳入版本库）；不要把个人密钥放入运行说明或分享压缩包。断网和配额不足会记录 LLM fallback；这种运行不能通过真实 LLM 录像验收，可先用 `nearest` 检查本地系统。

## 5. 验证、录制和生成报告

先完成同策略、同场景的不录制试跑，再运行回归并生成录制许可。以下以 Nearest 为例；真实 LLM 将两处 `--policy nearest` 改为 `--policy llm_guided`，并按上节配置环境：

```powershell
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-demo/dryrun
./data/venv-igibson/Scripts/python.exe scripts/run_integrated_checks.py --output runs/my-demo/checks/tests.log
./data/venv-igibson/Scripts/python.exe scripts/validate_integrated.py --run runs/my-demo/dryrun --tests runs/my-demo/checks/tests.log --output runs/my-demo/validation.json
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-demo/recording --record --validation runs/my-demo/validation.json
./data/venv-igibson/Scripts/python.exe scripts/check_integrated_artifacts.py runs/my-demo/recording
```

若录制的是零回退的真实 Gemini，再提取实际回答：

```powershell
./data/venv-igibson/Scripts/python.exe scripts/package_llm_demo.py runs/my-demo/recording --baseline runs/integrated-final-20261017/recording
```

该可选旧工具要求已有一份基线录像；没有基线时可直接使用下面的报告生成器，它会从事件日志提取模型回答。

```powershell
./data/venv-igibson/Scripts/python.exe scripts/build_demo_report.py --recording runs/my-demo/recording --output runs/my-demo/report
```

打开 `runs/my-demo/report/index.html`，可播放 15 fps MP4、点击决策/操作/反馈/验收时间跳转。录像保存在旁边的 `recording/`；分享时保留整个 `my-demo/` 目录的相对结构。源码、模型、策略、难度或推理设备改变后，需要重新做验证。报告生成器不需要旧版作者机器上的 14 项历史媒体。

也可以打包报告和本次证据，再把 ZIP 发给队友直接离线查看：

```powershell
python scripts/package_demo_report.py --run-root runs/my-demo --output runs/my-demo-report.zip
```

解压后打开 `my-demo/report/index.html`。打包器只收录报告、录像和运行证据，不收录虚拟环境、原始数据集或 `.env`。需要重新运行仿真的队友仍按首次安装步骤准备本机环境。

尚未提交的本地修改也可以通过源码交接包分享：

```powershell
python scripts/package_source_handoff.py --output runs/project-source.zip
```

队友解压 `frontier-search/` 后按本说明安装；无需复制原作者的绝对路径或虚拟环境。源码包不含 Git 历史、数据集、权重、录像或密钥，包含当前工作文件及 SHA256 清单。最新录像另用报告包分享。

## 6. Linux 与其他平台

项目的一键原生编译安装器目前只针对 Windows。Linux 路径受启动器支持，但本次没有第二台 Linux 机器进行端到端复现，不把它标为已验证平台。Linux 队友可按官方 iGibson 说明准备 C++、CMake、EGL/OpenGL 与显卡驱动，再建立 Python 3.8.20 环境并安装固定依赖，编译上述固定 iGibson revision，运行 `download_igibson_demo.py` 下载资产；检测依赖仍选对应的 requirements 文件。

将环境放在 `data/venv-igibson/` 或通过 `FRONTIER_PYTHON` 指定；之后相同 Python 启动命令和验证/录制流程适用。Linux 的 native Python 位于 `data/venv-igibson/bin/python`，替换 Windows 命令里的 `Scripts/python.exe`。macOS 完整仿真未验证；只查看 HTML/MP4 报告不要求安装仿真环境。

以下是 Ubuntu x86_64 的参考安装步骤，要求已准备好可用于 EGL/OpenGL 的显卡驱动。它不自动升级驱动，且尚未在本项目的第二台机器执行验证：

```bash
# 仓库根目录；先按 uv 官方说明安装 uv
sudo apt-get update
sudo apt-get install build-essential cmake git libegl-dev libgl1-mesa-dev libglib2.0-0 fonts-dejavu-core
uv venv --python 3.8.20 data/venv-igibson
uv pip install --python data/venv-igibson/bin/python -r requirements-integrated.txt -c constraints-windows-py38.txt --extra-index-url https://download.pytorch.org/whl/cu121 --index-strategy unsafe-best-match
git clone https://github.com/StanfordVL/iGibson.git data/igibson-src
git -C data/igibson-src checkout 7d9598278b95ec1039e1f4366c66b45346521e44
git -C data/igibson-src submodule update --init --recursive
# 复用同一处 RGB 纹理缓冲区修复；MSVC 专用补丁不用于 Linux
python3 -c "import sys; sys.path.insert(0, 'scripts'); from setup_igibson import patch, PATCHES; patch(*PATCHES[-1])"
CMAKE_POLICY_VERSION_MINIMUM=3.5 uv pip install --python data/venv-igibson/bin/python --no-build-isolation -e data/igibson-src -c constraints-windows-py38.txt
uv pip install --python data/venv-igibson/bin/python --no-deps -e .
data/venv-igibson/bin/python scripts/download_igibson_demo.py --root data/igibson-data
python3 -c "import sys; sys.path.insert(0, 'scripts'); from setup_project import download_model; download_model()"
data/venv-igibson/bin/python scripts/check_environment.py --render --output runs/environment-linux.json
python3 scripts/run_integrated.py --policy nearest --output runs/my-linux-dryrun
```

CPU 推理时将上述依赖文件换为 `requirements-integrated-cpu.txt`、Torch 索引换为 `/whl/cpu`。Linux 的 iGibson CMake 会根据本机是否存在 CUDA Toolkit 配置原生渲染扩展；完整背景和 EGL/CUDA 要求仍以链接的官方 iGibson 安装文档为准。先让 `check_environment.py --render` 在本机通过，再运行完整搜索和录制。

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| `No module named ...` | 使用 `data/venv-igibson` 的解释器；仅 `pip install -e .` 不会安装仿真器和检测依赖 |
| `.ps1` 被执行策略阻止 | 使用本文的 Python 启动器，不需要修改系统执行策略 |
| `VRRendererContext` / DLL 导入失败 | 检查 x64 Python、MSVC/SDK 和显卡驱动，重新执行安装器构建原生扩展 |
| 找不到 Rs / Locobot / mug | 重跑 `python scripts/download_igibson_demo.py --root data/igibson-data`，再运行环境自检；检查是否设置过指向其他资源的 `GIBSON_*` 环境变量 |
| YOLO 权重校验失败 | 保留异常文件用于排查，从官方 release 重新下载；不要通过关闭校验继续 |
| `No collision-free supported cart crossing` | 先同步最新 `cart_motion.py`，运行带 `--render` 的自检；不应删除碰撞检查来启动 |
| `validation ... stale` 或 detector runtime 改变 | 当前源码/权重/推理环境与验证不一致，重新运行试跑、回归和验证 |
| clone 后最新报告链接不存在 | 新生成的 `runs/` 默认被忽略；请向生成者取得最新报告包，或按本文重新生成 |
| 播放速度和计算速度不同 | 视频按仿真时钟播放，实际渲染/检测/模型调用耗时见 `wall_time_s`，不能将录像播放速度当成实时计算能力 |
