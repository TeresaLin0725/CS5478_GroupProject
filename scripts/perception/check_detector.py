"""Compare OpenCV Zoo detectors on rendered iGibson images. Used for offline evaluation of target detection and tracking."""

import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import statistics
import time


# Measure detections and latency on saved images without producing target claims.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, nargs="+", required=True)
    parser.add_argument("--detector", choices=("yolox", "nanodet"), default="yolox")
    parser.add_argument("--model-dir", type=Path)
    parser.add_argument("--target", default="cup")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    if args.model_dir is None:
        args.model_dir = Path("weights") / args.detector
    if args.output.exists() or args.repeats < 1:
        parser.error("choose a new output directory and at least one timing repeat")
    import cv2
    import numpy as np

    source = json.loads((args.model_dir / "source.json").read_text())
    model_path = args.model_dir / ("object_detection_%s_2022nov.onnx" % args.detector)
    model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if model_hash != source["sha256"]:
        raise ValueError("model hash disagrees with source metadata")
    # Read the official vocabulary as data, without executing the demo program.
    tree = ast.parse((args.model_dir / "demo.py").read_text())
    classes = next(ast.literal_eval(node.value) for node in tree.body
                   if isinstance(node, ast.Assign)
                   and any(isinstance(target, ast.Name) and target.id == "classes" for target in node.targets))
    spec = importlib.util.spec_from_file_location("opencv_zoo_" + args.detector,
                                                 str(args.model_dir / (args.detector + ".py")))
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)
    if args.detector == "yolox":
        detector = reference.YoloX(str(model_path), confThreshold=0.25, nmsThreshold=0.5,
                                   backendId=cv2.dnn.DNN_BACKEND_OPENCV, targetId=cv2.dnn.DNN_TARGET_CPU)
        input_size, nms_mode = 640, "class-aware"
    else:
        detector = reference.NanoDet(str(model_path), prob_threshold=0.25, iou_threshold=0.5,
                                     backend_id=cv2.dnn.DNN_BACKEND_OPENCV, target_id=cv2.dnn.DNN_TARGET_CPU)
        input_size, nms_mode = 416, "class-agnostic (official reference)"
    args.output.mkdir(parents=True)
    report = {
        "model": "OpenCV Zoo " + model_path.name, "detector": args.detector, "source": source,
        "python": platform.python_version(), "opencv": cv2.__version__, "numpy": np.__version__,
        "backend": "OpenCV CPU", "opencv_threads": cv2.getNumThreads(),
        "confidence_threshold": 0.25, "nms_iou_threshold": 0.5, "nms_mode": nms_mode,
        "network_input": [input_size, input_size],
        "preprocessing": "Official demo RGB and letterboxing; reference inference normalization",
        "target_class": args.target, "target_supported": args.target in classes,
        "target_class_id": classes.index(args.target) if args.target in classes else None,
        "class_names": classes, "images": [],
        "limitations": "Unlabelled screening: detections are not verified success, accuracy or multi-frame confirmation.",
    }
    for index, path in enumerate(args.images):
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError("could not load image: %s" % path)
        height, width = image.shape[:2]
        ratio = min(input_size / width, input_size / height)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        neww, newh = int(width * ratio), int(height * ratio)
        if args.detector == "yolox":
            blob = np.full((input_size, input_size, 3), 114., dtype=np.float32)
            resized = cv2.resize(rgb, (neww, newh)).astype(np.float32)
            blob[:newh, :neww] = resized
        else:
            left, top = (input_size - neww) // 2, (input_size - newh) // 2
            resized = cv2.resize(rgb, (neww, newh), interpolation=cv2.INTER_AREA)
            blob = cv2.copyMakeBorder(resized, top, input_size - newh - top,
                                      left, input_size - neww - left, cv2.BORDER_CONSTANT, value=0)
        started = time.perf_counter()
        predictions = detector.infer(blob)
        first_ms = (time.perf_counter() - started) * 1000
        timings = []
        for _ in range(args.repeats):
            started = time.perf_counter()
            detector.infer(blob)
            timings.append((time.perf_counter() - started) * 1000)
        detections = []
        for prediction in predictions:
            if args.detector == "yolox":
                x, y, w, h = [float(value / ratio) for value in prediction[:4]]
                x2, y2 = x + w, y + h
            else:
                x, y, x2, y2 = [float(value) for value in prediction[:4]]
                x, x2 = (x - left) * width / neww, (x2 - left) * width / neww
                y, y2 = (y - top) * height / newh, (y2 - top) * height / newh
            score, class_id = float(prediction[4]), int(prediction[5])
            box = [max(0., min(width, x)), max(0., min(height, y)),
                   max(0., min(width, x2)), max(0., min(height, y2))]
            if box[2] <= box[0] or box[3] <= box[1]:
                continue
            detections.append({"class_name": classes[class_id], "confidence": score, "bbox_xyxy_px": box})
            x1, y1, x2, y2 = [int(round(value)) for value in box]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 1)
            cv2.putText(image, "%s %.2f" % (classes[class_id], score), (x1, max(12, y1)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
        image_result = {
            "path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "size": [width, height], "first_inference_ms": first_ms,
            "warm_inference_ms": timings, "warm_median_ms": statistics.median(timings),
            "detections": detections,
            "target_counts": {str(threshold): sum(d["class_name"] == args.target and d["confidence"] >= threshold
                                                  for d in detections) for threshold in (0.25, 0.5, 0.8)},
            "annotated_image": "%02d_%s.png" % (index, path.stem),
        }
        cv2.imwrite(str(args.output / image_result["annotated_image"]), image)
        report["images"].append(image_result)
        print(path, [(d["class_name"], round(d["confidence"], 3)) for d in detections],
              "median %.1f ms" % statistics.median(timings), flush=True)
    (args.output / "results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
