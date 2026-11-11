# 48轮有噪声碗搜索实验存档

[打开中文报告](report-zh/bowl48-report-zh.html) · [Markdown版](report-zh/bowl48-report-zh.md)

本批次48轮全部完成，共45轮成功；LLM与IG均为12/12成功。报告包含成功率、时间、路径、两处目标与两个起点分析、四组四方法轨迹对比、配对不确定性和全部48轮明细。

仓库存档中的逐轮轨迹使用WebP预览；完整分辨率PNG保留在原实验目录。WebP仅用于展示，统计数据与逐轮JSON均直接复制自原结果。

原始protocol.json及其SHA256完整保留；其中机器路径属于实验时的来源记录，不能直接用作其他电脑的运行配置。图像是原始Rs平面图与原生渲染新增物体的合成，动态推车显示初始位置。原生资源和完整事件日志位于本机runs/bowl-48-noise-20261110-02目录。

复现报告存档：

```powershell
python scripts/build_bowl48_chinese_report.py --run-root runs/bowl-48-noise-20261110-02
python scripts/archive_bowl48_report.py --run-root runs/bowl-48-noise-20261110-02 --output report/bowl-48-noise-20261110-02
```
