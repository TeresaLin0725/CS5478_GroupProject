"""Rebuild editable SVG, vector PDF and 3x PNG framework figures.

Run from the workspace root with the task-local PDF dependencies:
  $env:PYTHONPATH="$PWD/tmp/figure-deps"
  py -3.10 output/pdf/project-framework/build_framework.py

All module arrows describe data forwarded by EpisodeRunner, not direct calls.
"""
from pathlib import Path
from html import escape
import json
import math
import os

from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.colors import HexColor
import pymupdf


HERE = Path(__file__).resolve().parent
W, H = 1680, 1040
FONT_DIR = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
for name, filename in (("CN", "simhei.ttf"), ("EN", "arial.ttf"),
                       ("EN-B", "arialbd.ttf")):
    pdfmetrics.registerFont(TTFont(name, str(FONT_DIR / filename)))

C = {
    "ink": "#20313F", "muted": "#536674", "line": "#536675",
    "border": "#BDCAD2", "panel": "#F6F8FA", "white": "#FFFFFF",
    "blue": "#216A8C", "blue_bg": "#EDF5F9", "blue_line": "#94B8C9",
    "purple": "#68548D", "purple_bg": "#F3F0F8", "purple_line": "#B9ABCF",
    "green": "#39735E", "green_bg": "#EFF6F2", "green_line": "#A2C2B3",
    "amber": "#9B682B", "amber_bg": "#FFFCF4", "amber_line": "#C49B61",
}


class Figure:
    def __init__(self, lang):
        self.lang = lang
        self.base = HERE / ("framework-" + lang)
        self.pdf = canvas.Canvas(str(self.base.with_suffix(".pdf")), pagesize=(W, H))
        self.pdf.setTitle("Embodied Indoor Object Search: Implemented Framework and Future Optimization")
        self.pdf.setAuthor("CS5478 Project")
        self.svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
                    '<title>Implemented closed-loop framework and future optimization directions</title>',
                    '<desc>Solid boxes show implemented modules. Dashed amber boxes show proposed optimization. All module data flows are coordinated by EpisodeRunner.</desc>']
        self.texts = []
        self.rect(0, 0, W, H, fill=C["white"], stroke=None)

    def rect(self, x, y, w, h, fill=None, stroke=None, radius=0, lw=1.5, dash=None):
        p = self.pdf
        p.saveState()
        p.setLineWidth(lw)
        if fill:
            p.setFillColor(HexColor(fill))
        if stroke:
            p.setStrokeColor(HexColor(stroke))
        if dash:
            p.setDash(dash)
        if radius:
            p.roundRect(x, H-y-h, w, h, radius, stroke=bool(stroke), fill=bool(fill))
        else:
            p.rect(x, H-y-h, w, h, stroke=bool(stroke), fill=bool(fill))
        p.restoreState()
        ds = f' stroke-dasharray="{",".join(map(str, dash))}"' if dash else ""
        self.svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" fill="{fill or "none"}" stroke="{stroke or "none"}" stroke-width="{lw}"{ds}/>')

    def path(self, points, color=None, lw=1.7, dash=None, arrow=False):
        color = color or C["line"]
        p = self.pdf
        p.saveState()
        p.setStrokeColor(HexColor(color))
        p.setLineWidth(lw)
        p.setLineJoin(1)
        if dash:
            p.setDash(dash)
        q = p.beginPath()
        q.moveTo(points[0][0], H-points[0][1])
        for x, y in points[1:]:
            q.lineTo(x, H-y)
        p.drawPath(q)
        p.restoreState()
        ds = f' stroke-dasharray="{",".join(map(str,dash))}"' if dash else ""
        d = "M " + " L ".join(f"{x},{y}" for x,y in points)
        self.svg.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{lw}" stroke-linejoin="round"{ds}/>')
        if arrow:
            x,y = points[-1]
            x0,y0 = points[-2]
            dx,dy = x-x0,y-y0
            norm = math.hypot(dx,dy)
            dx,dy = dx/norm,dy/norm
            size = 8.5
            vertices = [(x,y), (x-size*dx+4*dy,y-size*dy-4*dx),
                        (x-size*dx-4*dy,y-size*dy+4*dx)]
            p.saveState()
            p.setFillColor(HexColor(color))
            q = p.beginPath()
            q.moveTo(vertices[0][0], H-vertices[0][1])
            for xx,yy in vertices[1:]:
                q.lineTo(xx,H-yy)
            q.close()
            p.drawPath(q,stroke=0,fill=1)
            p.restoreState()
            self.svg.append(f'<polygon points="{" ".join(f"{a},{b}" for a,b in vertices)}" fill="{color}"/>')

    def text(self, x, y, value, size=22, color=None, bold=False, anchor="start", max_width=None):
        """Top-aligned editable text; the same widths drive both exports."""
        font = "CN" if any(ord(c)>255 for c in value) else ("EN-B" if bold else "EN")
        glyphs = pdfmetrics.getFont(font).face.charToGlyph
        missing = [c for c in value if not c.isspace() and ord(c) not in glyphs]
        if missing:
            raise ValueError(f"Missing glyphs in {font}: {missing!r}")
        if max_width:
            actual = pdfmetrics.stringWidth(value,font,size)
            if actual > max_width:
                size *= max_width / actual
        color = color or C["ink"]
        baseline = y+size*.81
        p = self.pdf
        p.setFont(font,size)
        p.setFillColor(HexColor(color))
        draw = {"start":p.drawString,"middle":p.drawCentredString,"end":p.drawRightString}[anchor]
        draw(x,H-baseline,value)
        family = "SimHei, Microsoft YaHei, Arial, sans-serif" if font=="CN" else "Arial, sans-serif"
        self.svg.append(f'<text x="{x}" y="{baseline:.3f}" font-family="{family}" font-size="{size:.3f}" font-weight="{"700" if bold else "400"}" text-anchor="{anchor}" fill="{color}">{escape(value)}</text>')
        self.texts.append(dict(text=value,size=round(size,2),x=x,y=y))

    def lines(self, x, y, values, size=22, leading=33, color=None, max_width=None):
        for i,value in enumerate(values):
            self.text(x,y+i*leading,value,size,color,max_width=max_width)

    def label(self, x, y, value, size=18, color=None, pad=8):
        font = "CN" if any(ord(c)>255 for c in value) else "EN"
        width = pdfmetrics.stringWidth(value,font,size)
        self.rect(x-width/2-pad,y-3,width+2*pad,size+8,C["white"])
        self.text(x,y,value,size,color,anchor="middle")

    def node(self,x,y,w,h,title,subtitle,color,bg,lines):
        self.rect(x,y,w,h,bg,color,8,lw=1.8)
        self.text(x+20,y+20,title,27,color,bold=True,max_width=w-40)
        self.text(x+20,y+58,subtitle,17,C["muted"],max_width=w-40)
        self.path([(x+20,y+90),(x+w-20,y+90)],color,lw=1.1)
        self.lines(x+20,y+112,lines,23,38,max_width=w-40)

    def finish(self):
        self.pdf.showPage()
        self.pdf.save()
        self.svg.append('</svg>')
        self.base.with_suffix('.svg').write_text('\n'.join(self.svg),encoding='utf-8')
        with pymupdf.open(self.base.with_suffix('.pdf')) as doc:
            page=doc[0]
            pix=page.get_pixmap(matrix=pymupdf.Matrix(3,3),alpha=False)
            pix.save(self.base.with_suffix('.png'))
            preview=page.get_pixmap(matrix=pymupdf.Matrix(1,1),alpha=False)
            preview.save(HERE / (self.base.stem+'-preview.png'))
        return {"language":self.lang,"page_size":[W,H],"png_size":[W*3,H*3],
                "minimum_font_size":min(t['size'] for t in self.texts),
                "text_items":len(self.texts),"files":[self.base.with_suffix(e).name for e in ['.svg','.pdf','.png']]}


def build(lang):
    z = lang == 'zh'
    f=Figure(lang)
    tr=lambda cn,en:cn if z else en
    # Title and status legend; status is redundant in border style and text.
    f.text(42,30,tr('具身室内目标搜索：已实现框架与未来优化','Embodied Indoor Object Search: Framework and Future Optimization'),32,bold=True,max_width=1190)
    f.text(44,77,tr('目标驱动的感知、搜索决策与导航执行闭环','A target-driven closed loop of perception, search and navigation'),21,C['muted'])
    f.rect(1280,34,30,19,C['white'],C['line'],2)
    f.text(1320,32,tr('已实现','Implemented'),19)
    f.rect(1452,34,30,19,C['amber_bg'],C['amber'],2,dash=[5,4])
    f.text(1492,32,tr('待优化','Future work'),19,C['amber'])

    # Inputs are observations/allowed priors, never evaluator-only ground truth.
    f.rect(42,126,224,77,C['panel'],C['border'],7)
    f.text(154,141,tr('任务输入','Task input'),21,bold=True,anchor='middle')
    f.text(154,173,tr('目标类别：cup','Target class: cup'),20,anchor='middle')
    f.rect(330,126,1308,77,C['panel'],C['border'],7)
    f.text(355,140,tr('允许的先验与公共配置','Allowed priors and shared configuration'),22,bold=True)
    f.text(355,174,tr('已知机器人位姿 / 相机标定 / 区域几何先验 / 机器人足迹与预算','Known pose / camera calibration / region geometry / robot footprint and budgets'),21,C['muted'],max_width=1240)
    f.path([(154,203),(154,216),(370,216),(370,229)],arrow=True,lw=1.4)
    f.path([(984,203),(984,229)],arrow=True,lw=1.4)

    # Sole coordinator. The three modules exchange data only through this runner.
    f.rect(306,229,1332,422,C['panel'],C['border'],9,lw=1.6)
    f.text(330,246,tr('统一运行与编排  EpisodeRunner','EpisodeRunner: lifecycle and message coordination'),25,bold=True)
    f.text(330,281,tr('生命周期与预算监督 · 新鲜帧/状态校验 · 冷却、重试与有界恢复','Budgets and watchdogs / fresh-state validation / cooldowns and bounded recovery'),19,C['muted'],max_width=1250)

    # Platform: conceptual vector floor plan, not a fabricated sensor image.
    f.rect(42,326,224,272,C['white'],C['line'],8,lw=1.8)
    f.text(154,347,tr('仿真与机器人','Simulation / robot'),24,bold=True,anchor='middle',max_width=196)
    f.text(154,386,'iGibson Rs',23,anchor='middle')
    f.text(154,419,'Locobot',23,anchor='middle')
    f.rect(85,465,139,90,C['panel'],C['border'],2)
    f.path([(135,465),(135,504),(151,504)],C['border'],2)
    f.path([(135,526),(135,555)],C['border'],2)
    f.path([(174,465),(174,490),(224,490)],C['border'],2)
    f.rect(98,511,22,28,C['blue_bg'],C['blue'],5)
    f.path([(109,512),(109,493)],C['blue'],1.8,arrow=True)
    f.text(154,570,tr('原生 RGB-D 与物理步进','Native RGB-D + physics'),15,C['muted'],anchor='middle',max_width=204)

    # A: concise ownership-faithful perception stack.
    f.node(330,326,350,272,tr('A  感知与建图','A  Perception & Mapping'),
           'RGBDMapper',C['blue'],C['blue_bg'],[
               tr('RGB-D 融合与占据建图','RGB-D fusion / occupancy'),
               tr('前沿提取与信息增益','Frontiers / information gain'),
               tr('YOLO11s 检测与多帧跟踪','YOLO11s / multi-frame tracks'),
               tr('目标定位与确认声明','Target localization / claims')])

    # C: the four policies are alternatives, not four serial processing stages.
    x,y,w,h=782,326,390,272
    f.rect(x,y,w,h,C['purple_bg'],C['purple'],8,lw=1.8)
    f.text(x+20,y+20,tr('C  搜索策略','C  Search Policy'),27,C['purple'],bold=True)
    f.text(x+20,y+58,'SearchPolicy',17,C['muted'])
    f.path([(x+20,y+90),(x+w-20,y+90)],C['purple'],1.1)
    f.text(x+20,y+110,tr('仅在合格候选集合中选择','Select from eligible candidates'),22,max_width=w-40)
    for px,py,pw,label in [(x+20,y+150,167,'Random'),(x+204,y+150,166,'Nearest'),
                            (x+20,y+194,167,'Information Gain'),(x+204,y+194,166,'LLM-guided')]:
        f.rect(px,py,pw,34,C['white'],C['purple_line'],4,lw=1.0)
        f.text(px+pw/2,py+8,label,18,C['purple'],anchor='middle',max_width=pw-12)
    f.text(x+w/2,y+244,tr('目标已观测时优先接近确认','Observed target: approach first'),19,C['muted'],anchor='middle',max_width=w-30)

    f.node(1290,326,324,272,tr('B  导航与执行','B  Navigation'),
           'TeamNavigator',C['green'],C['green_bg'],[
               tr('可达性与路径代价评估','Reachability / path cost'),
               tr('足迹代价图与 A* 规划','Footprint costmap / A*'),
               tr('轮速执行、避障与扫描','Drive / avoidance / scans'),
               tr('目标接近与停稳观察','Approach / stationary views')])

    # Logical data flow, all brokered by EpisodeRunner.
    f.path([(266,457),(330,457)],arrow=True,lw=2)
    f.label(298,427,'RGB-D',17,pad=2)
    f.text(298,473,tr('位姿','Pose'),17,C['muted'],anchor='middle')
    f.path([(680,456),(782,456)],arrow=True,lw=2)
    f.label(731,423,tr('世界状态','WorldState'),18,pad=2)
    f.path([(1172,413),(1290,413)],arrow=True,lw=2)
    f.label(1231,382,tr('子目标','Subgoal'),18)
    f.path([(1290,521),(1172,521)],arrow=True,lw=2)
    f.label(1231,487,tr('代价 / 反馈','Cost / status'),17)

    # Close the action-observation loop without crossing the verification channel.
    f.path([(1452,598),(1452,625),(225,625),(225,598)],arrow=True,lw=1.9)
    f.label(925,613,tr('轮速控制与观测动作；运动中持续采集并更新地图','Wheel-speed and observation commands; perception continues during motion'),19)
    f.text(330,666,tr('图中模块间箭头均表示由 Runner 转发的数据流。','All inter-module arrows denote data forwarded by the runner.'),18,C['muted'])

    # Isolated terminal verifier. Results never re-enter the search policy.
    f.path([(95,598),(95,716)],arrow=True,lw=1.5)
    f.label(166,686,tr('私有真值通道','Private ground truth'),16)
    f.path([(1532,651),(1532,716)],arrow=True,lw=1.5)
    f.label(1406,686,tr('终止声明 + 冻结证据','Terminal claim + evidence'),17)
    f.rect(42,716,1596,100,C['white'],C['line'],7,lw=1.6)
    f.text(64,734,tr('独立目标验收','Independent verification'),25,bold=True,max_width=312)
    f.text(64,775,tr('终止后评分，结果仅写日志','Terminal scoring; log output only'),18,C['muted'],max_width=312)
    f.path([(401,736),(401,796)],C['border'],lw=1.1)
    f.lines(428,735,[tr('真实目标实例匹配 · 不同采集时刻的证据','Target-instance matching / evidence from distinct captures'),
                    tr('近距离与停稳检查 · 验收结果不回传搜索策略','Proximity and stillness checks / no verifier feedback to policy')],21,36,max_width=780)
    f.path([(1254,736),(1254,796)],C['border'],lw=1.1)
    f.text(1280,734,tr('实验记录与评测','Logs & evaluation'),23,bold=True,max_width=330)
    f.text(1280,775,tr('成功率 / 时间 / 距离 / 失败','Success / time / path / failures'),18,C['muted'],max_width=330)

    # Future-work boxes are separate from the implemented pipeline.
    f.text(42,849,tr('后续优化方向','Proposed optimization directions'),23,C['amber'],bold=True)
    for x,w,title,items in [
        (42,772,tr('搜索优化  |  C','Search optimization  |  C'),[
            tr('语义线索、信息增益与路径 / 失败代价联合评分','Joint scoring: semantics, gain, path cost and failure history'),
            tr('Top-K 候选压缩与更丰富的跨决策搜索记忆','Top-K candidate compression and richer search memory')]),
        (844,794,tr('双向通讯与恢复优化  |  C / B','Bidirectional feedback & recovery  |  C / B'),[
            tr('更细的失败原因与局部几何证据反馈','Richer failure reasons and local geometric evidence'),
            tr('反馈驱动的替代视点、目标接近与重规划增强','Feedback-guided viewpoints, target approach and replanning')]),
    ]:
        f.rect(x,886,w,126,C['amber_bg'],C['amber'],7,lw=1.8,dash=[7,5])
        f.text(x+22,903,title,24,C['amber'],bold=True,max_width=w-44)
        f.lines(x+22,946,items,21,32,max_width=w-44)
    return f.finish()


if __name__=='__main__':
    HERE.mkdir(parents=True,exist_ok=True)
    result=[build(lang) for lang in ['zh','en']]
    (HERE/'figure-metadata.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
