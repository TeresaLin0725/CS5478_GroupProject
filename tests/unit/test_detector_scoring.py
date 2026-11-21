import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.perception.score_furnished_probes import main


class FurnishedScoringTests(unittest.TestCase):
    def test_background_labels_and_hidden_targets_do_not_count_as_matches(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            labels, predictions = [], []
            for name, present, pixels, boxes in (
                ("visible.png", True, 100, [(0.9, [0, 0, 10, 10]), (0.4, [20, 20, 30, 30])]),
                ("missed.png", True, 100, [(0.95, [20, 20, 30, 30])]),
                ("hidden.png", True, 0, []),
                ("absent.png", False, 0, [(0.3, [20, 20, 30, 30])]),
            ):
                data = name.encode()
                (root / name).write_bytes(data)
                labels.append({"image": name, "target_present": present,
                               "target_visible_pixels": pixels,
                               "visible_bbox_xyxy_px": [0, 0, 10, 10] if pixels else None})
                predictions.append({"path": str(root / name), "sha256": hashlib.sha256(data).hexdigest(),
                                    "detections": [{"class_name": "cup", "confidence": confidence,
                                                    "bbox_xyxy_px": box} for confidence, box in boxes]})
            (root / "manifest.json").write_text(json.dumps({"images": labels}))
            (root / "results.json").write_text(json.dumps({"target_class": "cup", "images": predictions}))
            argv = ["score", "--manifest", str(root / "manifest.json"), "--results", str(root / "results.json"),
                    "--output", str(root / "scored.json")]
            with patch("sys.argv", argv), contextlib.redirect_stdout(io.StringIO()):
                main()
            report = json.loads((root / "scored.json").read_text())
            self.assertEqual(report["visible_target_views"], 2)
            self.assertEqual(report["fully_hidden_target_views"], 1)
            self.assertEqual(report["absent_views"], 1)
            self.assertEqual(report["thresholds"]["0.25"], {
                "visible_views_with_matched_cup": 1,
                "absent_views_with_cup_prediction": 1,
                "visible_views_with_unmatched_cup_prediction": 2,
            })
            self.assertEqual(report["thresholds"]["0.8"], {
                "visible_views_with_matched_cup": 1,
                "absent_views_with_cup_prediction": 0,
                "visible_views_with_unmatched_cup_prediction": 1,
            })
            for duplicate_labels, duplicate_predictions in ((True, False), (False, True)):
                (root / "manifest.json").write_text(json.dumps({
                    "images": labels + [labels[0]] if duplicate_labels else labels}))
                (root / "results.json").write_text(json.dumps({"target_class": "cup",
                    "images": predictions + [predictions[0]] if duplicate_predictions else predictions}))
                with patch("sys.argv", argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main()
            (root / "manifest.json").write_text(json.dumps({"images": labels}))
            (root / "results.json").write_text(json.dumps({"target_class": "cup", "images": predictions}))
            (root / "visible.png").write_bytes(b"changed capture")
            with patch("sys.argv", argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main()


if __name__ == "__main__":
    unittest.main()
