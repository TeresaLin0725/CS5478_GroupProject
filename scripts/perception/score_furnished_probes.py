"""Compare cup predictions with visible-object boxes from furnished probe masks. Used for offline evaluation of target detection and tracking."""

import argparse
import hashlib
import json
from pathlib import Path


# Measure how closely two image boxes overlap.
def box_iou(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0.0


# Separate localized target detections from background labels and hidden targets.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    results = json.loads(args.results.read_text(encoding="utf-8"))
    if results["target_class"] != "cup":
        parser.error("results must report the cup class")
    predictions = {Path(row["path"]).name: row for row in results["images"]}
    names = {row["image"] for row in manifest["images"]}
    if len(predictions) != len(results["images"]) or len(names) != len(manifest["images"]):
        parser.error("image names must be unique in results and manifest")
    if set(predictions) != names:
        parser.error("results and manifest must contain the same images")
    rows = []
    for label in manifest["images"]:
        prediction = predictions[label["image"]]
        path = args.manifest.parent / label["image"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != prediction["sha256"]:
            parser.error("image hash mismatch: " + str(path))
        target_box = label.get("visible_bbox_xyxy_px")
        cups = [{"confidence": d["confidence"], "iou_with_visible_target": box_iou(d["bbox_xyxy_px"], target_box) if target_box else 0.0,
                 "bbox_xyxy_px": d["bbox_xyxy_px"]}
                for d in prediction["detections"] if d["class_name"] == "cup"]
        rows.append({"image": label["image"], "target_present": label["target_present"],
                     "target_visible_pixels": label["target_visible_pixels"], "cup_predictions": cups})
    visible = [r for r in rows if r["target_present"] and r["target_visible_pixels"] > 0]
    absent = [r for r in rows if not r["target_present"]]
    hidden = [r for r in rows if r["target_present"] and r["target_visible_pixels"] == 0]
    summary = {}
    for threshold in (0.25, 0.5, 0.8):
        summary[str(threshold)] = {
            "visible_views_with_matched_cup": sum(any(d["confidence"] >= threshold and d["iou_with_visible_target"] >= 0.5 for d in r["cup_predictions"]) for r in visible),
            "absent_views_with_cup_prediction": sum(any(d["confidence"] >= threshold for d in r["cup_predictions"]) for r in absent),
            "visible_views_with_unmatched_cup_prediction": sum(any(d["confidence"] >= threshold and d["iou_with_visible_target"] < 0.5 for d in r["cup_predictions"]) for r in visible),
        }
    report = {"visible_target_views": len(visible), "fully_hidden_target_views": len(hidden),
              "absent_views": len(absent), "matching_iou": 0.5, "thresholds": summary, "images": rows,
              "limitations": "Matches use visible-mask bounding boxes, not amodal boxes. Small correlated diagnostic sample; not benchmark recall or search success."}
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != "images"}, indent=2))


if __name__ == "__main__":
    main()
