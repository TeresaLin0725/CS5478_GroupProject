"""RGB-only detector boundary and single-frame depth localization.

Boxes use integer, half-open image bounds. Positions estimate a visible surface,
not an object's centre: median valid depth in the central half of the box is
projected along that patch's centre ray. Background/occlusion can bias this
estimate; no valid depth leaves position explicitly unknown.
"""

from dataclasses import dataclass
from math import ceil, floor, isfinite
from pathlib import Path
from statistics import median
from struct import unpack_from
from typing import Optional, Protocol, Tuple

from ..config import MappingConfig
from ..contracts import Point3, RGBDObservation
from ..sensors import ImageBuffer
from ..validation import is_finite_number, require


@dataclass(frozen=True)
class RGBDetection:
    class_name: str
    confidence: float
    bbox_xyxy_px: Tuple[int, int, int, int]


class RGBDetector(Protocol):
    def detect(self, image: ImageBuffer) -> Tuple[RGBDetection, ...]: ...


def validate_detection(detection: RGBDetection, image: ImageBuffer) -> None:
    require(isinstance(detection, RGBDetection), "detector must return RGBDetection records")
    require(isinstance(detection.class_name, str) and bool(detection.class_name.strip()),
            "detection requires a class name")
    require(is_finite_number(detection.confidence) and 0 <= detection.confidence <= 1,
            "invalid detection confidence")
    box = detection.bbox_xyxy_px
    require(isinstance(box, tuple) and len(box) == 4 and all(type(v) is int for v in box),
            "detection box must contain four integer coordinates")
    x1, y1, x2, y2 = box
    require(0 <= x1 < x2 <= image.width and 0 <= y1 < y2 <= image.height,
            "detection box outside image or empty")


def localize_detection(detection: RGBDetection, observation: RGBDObservation,
                       cfg: MappingConfig) -> Optional[Point3]:
    """Use aligned axial depth and the capture-time camera transform."""
    x1, y1, x2, y2 = detection.bbox_xyxy_px
    dx, dy = (x2 - x1) // 4, (y2 - y1) // 4
    left, right, top, bottom = x1 + dx, x2 - dx, y1 + dy, y2 - dy
    depths = []
    for v in range(top, bottom):
        for u in range(left, right):
            depth = unpack_from('<f', observation.depth_image.data,
                                4 * (v * observation.depth_image.width + u))[0]
            if isfinite(depth) and cfg.depth_min_m <= depth <= cfg.depth_max_m:
                depths.append(depth)
    if not depths:
        return None
    depth = median(depths)
    camera = observation.camera_intrinsics
    u, v = (left + right - 1) / 2, (top + bottom - 1) / 2
    point = ((u - camera.cx) * depth / camera.fx,
             (v - camera.cy) * depth / camera.fy, depth)
    world = tuple(sum(row[i] * point[i] for i in range(3)) + row[3]
                  for row in observation.camera_to_world.matrix[:3])
    require(all(isfinite(value) for value in world), "object projection overflowed world coordinates")
    return Point3(*world)


class YOLODetector:
    """Optional Ultralytics adapter; requires an existing local weights file.

    Imports are deferred until construction so mapping-only runs do not require
    ML packages. The mapper applies its configured confidence threshold.
    """

    def __init__(self, weights: str, *, device: str = 'cpu', minimum_confidence: float = 0.0) -> None:
        path = Path(weights)
        if not path.is_file():
            raise FileNotFoundError("detector weights must exist locally: %s" % path)
        from ultralytics import YOLO
        self._model = YOLO(str(path))
        self._device = device
        self._minimum_confidence = minimum_confidence

    def detect(self, image: ImageBuffer) -> Tuple[RGBDetection, ...]:
        import numpy as np
        require(image.encoding == 'rgb8' and image.width > 0 and image.height > 0
                and len(image.data) == image.width * image.height * 3,
                "YOLO requires packed rgb8 input")
        rgb = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
        # Ultralytics numpy sources use BGR; copy to own a contiguous buffer.
        bgr = rgb[:, :, ::-1].copy()
        result = self._model.predict(bgr, imgsz=640, conf=self._minimum_confidence, iou=0.5,
            device=self._device, rect=False, half=False, agnostic_nms=False,
            max_det=300, verbose=False, save=False)[0]
        detections = []
        for box, score, class_id in zip(result.boxes.xyxy.tolist(),
                result.boxes.conf.tolist(), result.boxes.cls.tolist()):
            require(len(box) == 4 and all(is_finite_number(v) for v in box),
                    "YOLO returned an invalid box")
            x1, y1, x2, y2 = box
            bounds = (max(0, floor(x1)), max(0, floor(y1)),
                      min(image.width, ceil(x2)), min(image.height, ceil(y2)))
            if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
                continue
            detection = RGBDetection(self._model.names[int(class_id)], float(score), bounds)
            validate_detection(detection, image)
            detections.append(detection)
        return tuple(detections)


class TiledYOLODetector(YOLODetector):
    """RGB-only small-target supplement; overlapping fixed tiles, then class NMS.

    The target class is a task input. Tiles never use geometry, instance IDs,
    localization estimates or evaluator records. Full-frame semantics remain.
    """
    def __init__(self,weights,*,target_class,device='cpu',tile_every_frames=1,minimum_confidence=0.0):
        super().__init__(weights,device=device,minimum_confidence=minimum_confidence)
        self.target_class=target_class
        self.tile_every_frames=tile_every_frames
        self._tile_calls=0
        self._inspection_mode=False

    def set_inspection_mode(self, enabled: bool) -> None:
        """Use fresh tiled evidence on every close-inspection capture."""
        self._inspection_mode=enabled

    def detect(self,image):
        import numpy as np
        result=list(super().detect(image))
        self._tile_calls=getattr(self,'_tile_calls',0)+1
        if (not getattr(self,'_inspection_mode',False) and
                (self._tile_calls-1) % getattr(self,'tile_every_frames',1)):
            return tuple(result)
        rgb=np.frombuffer(image.data,dtype=np.uint8).reshape(image.height,image.width,3)
        side=min(image.width,image.height)*2//3
        for y in sorted({0,image.height-side}):
            for x in sorted({0,(image.width-side)//2,image.width-side}):
                tile=np.ascontiguousarray(rgb[y:y+side,x:x+side])
                for item in YOLODetector.detect(self,ImageBuffer(side,side,'rgb8',tile.tobytes())):
                    if item.class_name==self.target_class and item.confidence>=.10:
                        a,b,c,d=item.bbox_xyxy_px
                        result.append(RGBDetection(item.class_name,item.confidence,(x+a,y+b,x+c,y+d)))
        kept=[]
        for item in sorted(result,key=lambda d:-d.confidence):
            a=item.bbox_xyxy_px
            duplicate=False
            for old in kept:
                if old.class_name!=item.class_name:continue
                b=old.bbox_xyxy_px
                overlap=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
                union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-overlap
                if overlap/union>.5:duplicate=True;break
            if not duplicate:kept.append(item)
        return tuple(kept)
