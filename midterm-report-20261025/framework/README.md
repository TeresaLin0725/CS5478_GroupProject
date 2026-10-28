# 项目框架图 / Project framework figure

这张图用于中期报告，区分当前已实现的搜索闭环和后续优化方向。

## 文件

| 文件 | 用途 |
| --- | --- |
| `framework-zh.png` | 中文高清图，5040 × 3120，可直接插入报告或幻灯片 |
| `framework-zh.svg` | 中文矢量图，文字与图形可编辑 |
| `framework-zh.pdf` | 中文矢量 PDF，嵌入字体，适合 LaTeX 与论文排版 |
| `framework-en.png` | 对应英文高清图 |
| `framework-en.svg` | 对应英文可编辑矢量图 |
| `framework-en.pdf` | 对应英文矢量 PDF |
| `build_framework.py` | 双语生成脚本，统一生成 PDF、SVG，再从最终 PDF 渲染 PNG |

`*-preview.png` 是 1680 × 1040 的检查预览，正式插图请使用不带 `-preview` 的 PNG。

## 图注（中文）

**图 X. 具身室内目标搜索的系统框架及后续优化方向。**
实线框表示中期已实现的模块与数据流。统一运行器协调 RGB-D 感知与建图（A）、搜索策略（C）和导航执行（B）：感知模块生成占据地图、前沿及语义目标轨迹，导航模块评估候选可达性与路径代价，搜索策略在合格候选集合中选择子目标，导航执行与持续感知构成闭环。四种策略共享感知、导航及独立验收，只在候选选择规则上不同。独立验收通过私有真值通道检查终止声明，其结果仅用于日志与评测。虚线框表示后续拟研究的搜索评分与候选压缩，以及失败证据反馈、替代观察位姿和目标接近重规划等优化方向。

## Caption (English)

**Figure X. Implemented framework and future optimization directions for embodied indoor object search.** Solid boxes denote implemented components. The episode runner coordinates perception and mapping (A), search policies (C), and navigation (B). RGB-D perception produces occupancy, frontier and semantic-track information; navigation assesses reachability and path cost; the policy selects an eligible subgoal; execution and continuous perception close the loop. All four policies share perception, navigation and independent verification, differing only in candidate selection. The terminal verifier uses a private ground-truth channel, and its results are used only for logging and evaluation. Dashed boxes denote proposed improvements to search scoring, candidate compression, failure feedback, alternative viewpoints and target-approach replanning.

## 阅读与编辑说明

- A / B / C 沿用现有项目模块名称；图按感知、决策、执行的阅读顺序排为 A → C → B。
- 所有模块间箭头表示由 `EpisodeRunner` 转发的逻辑数据流。模块不直接调用彼此。
- C → B 的子目标下发、B → C 的路径代价和执行反馈已经实现。虚线框指的是反馈精细度与反馈利用方式的增强。
- 当前已实现共享的失败排除、冷却、有界重试和恢复。虚线框中的搜索记忆指进一步丰富的结构化历史，不表示当前没有历史信息。
- 目标被检测到后，现有策略优先接近确认；“看见目标”与“独立验收成功”分开表示。
- 已知机器人位姿、区域几何和相机标定属于现有允许先验。图未声称已经实现 SLAM，也未将隐藏目标位置或房间语义标签作为策略输入。
- 图中的优化项是研究方向，尚未作为已实现能力或性能收益展示。目标接近失败的改进应优先结合当前中期报告中的问题分析验证。
- 推荐在论文中跨双栏或以接近整页宽度插入。SVG 的文字保持可编辑，中文字体优先使用 SimHei / Microsoft YaHei；PDF 已嵌入字体。

## 内容依据

核对了以下本地材料及相关接口：

- `../../../midterm-report-20261025/report/Midterm-Report-EN.pdf`，第 2、3、7 节。
- `../../../integration/src/frontier_search/runner/episode.py`
- `../../../integration/src/frontier_search/runner/integrated.py`
- `../../../integration/src/frontier_search/policies/llm.py`
- `../../../integration/src/frontier_search/policies/baselines.py`
- `../../../integration/src/frontier_search/navigation/api.py`
- `../../../integration/Documentation/english/interface_freeze.md`
- `../../../integration/Documentation/english/close_object_search.md`

图不包含成功率数值，避免把早期文档结果与最新中期报告实验批次混用。

## 重新生成

在项目根目录运行（生成依赖保存在临时目录，未修改项目运行环境）：

```powershell
$env:PYTHONPATH="$PWD/tmp/figure-deps"
py -3.10 output/pdf/project-framework/build_framework.py
```

需要 Python 3.10+、`reportlab`、`pymupdf`，以及 Windows 的 `simhei.ttf`、`arial.ttf`、`arialbd.ttf`。
