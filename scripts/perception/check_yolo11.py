"""Screen YOLO11 on saved iGibson images. Used for offline evaluation of target detection and tracking."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import time


# Measure YOLO11 detections and CPU latency without changing the mapper or simulator.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, nargs="+", required=True)
    parser.add_argument("--model", type=Path, default=Path("weights/yolo11/yolo11s.pt"))
    parser.add_argument("--target", default="cup")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    if args.output.exists() or args.repeats < 1 or not args.model.is_file():
        parser.error("use an existing model, a new output directory, and at least one repeat")
    os.environ.setdefault("YOLO_CONFIG_DIR", str(Path("runs/yolo-config").resolve()))
    Path(os.environ["YOLO_CONFIG_DIR"]).mkdir(parents=True, exist_ok=True)
    import cv2
    import numpy as np
    import torch
    import ultralytics
    from ultralytics import YOLO

    model = YOLO(str(args.model))
    options = dict(imgsz=640, conf=0.25, iou=0.5, device="cpu", rect=False,
                   half=False, agnostic_nms=False, max_det=300, verbose=False, save=False)
    report = {
        "model": args.model.name, "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
        "python": platform.python_version(), "ultralytics": ultralytics.__version__,
        "torch": torch.__version__, "opencv": cv2.__version__, "numpy": np.__version__,
        "backend": "PyTorch CPU", "options": options, "class_names": model.names,
        "target_class": args.target, "target_supported": args.target in model.names.values(),
        "images": [], "limitations": "Appearance screening, not localization-scored accuracy or search success.",
    }
    args.output.mkdir(parents=True)
    for index, path in enumerate(args.images):
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError("could not read image: %s" % path)
        started = time.perf_counter()
        result = model.predict(image, **options)[0]
        first_ms = (time.perf_counter() - started) * 1000
        timings, network_timings = [], []
        for _ in range(args.repeats):
            started = time.perf_counter()
            repeat = model.predict(image, **options)[0]
            timings.append((time.perf_counter() - started) * 1000)
            network_timings.append(float(repeat.speed["inference"]))
        detections = [
            {"class_name": model.names[int(class_id)], "confidence": float(score),
             "bbox_xyxy_px": [float(x) for x in box]}
            for box, score, class_id in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(),
                                            result.boxes.cls.tolist())
        ]
        annotation = "%02d_%s.png" % (index, path.stem)
        cv2.imwrite(str(args.output / annotation), result.plot())
        report["images"].append({
            "path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "size": [image.shape[1], image.shape[0]], "detections": detections,
            "first_predict_ms": first_ms, "warm_predict_ms": timings,
            "warm_predict_median_ms": statistics.median(timings),
            "warm_network_ms": network_timings,
            "torch_threads": torch.get_num_threads(),
            "target_counts": {str(threshold): sum(d["class_name"] == args.target and d["confidence"] >= threshold
                                                  for d in detections) for threshold in (0.25, 0.5, 0.8)},
            "annotated_image": annotation,
        })
        print(path.name, [(d["class_name"], round(d["confidence"], 3)) for d in detections],
              "median predict %.1f ms" % statistics.median(timings), flush=True)
    (args.output / "results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
