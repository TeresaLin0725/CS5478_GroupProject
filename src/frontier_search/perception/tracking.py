"""Deterministic spatial association for observed objects in a static scene.

Localized tracks persist for the episode. Unknown-depth detections remain
single-frame records: image overlap alone is insufficient across camera motion.
Matching is class-gated, one-to-one, nearest-edge greedy with stable tie breaks.
The first measured position also gates matches to prevent gradual spatial drift.
"""

from math import hypot
from statistics import median

from ..contracts import DetectionEvidence, ObjectTrack, Point3
from .detection import localize_detection, validate_detection


def _distance(a, b):
    return hypot(hypot(a.x_m - b.x_m, a.y_m - b.y_m), a.z_m - b.z_m)


def update_tracks(previous_tracks, previous_evidence, detections, observation,
                  cfg, track_counter, observation_counter):
    """Return new dictionaries/counters; never mutate a previous snapshot."""
    tracks = {key: track for key, track in previous_tracks.items()
              if track.position_world is not None}
    evidence = {key: item for key, item in previous_evidence.items()
                if item.track_id in tracks}
    # Collapse identical class/box proposals to the strongest confidence.
    unique = {}
    for detection in detections:
        validate_detection(detection, observation.color_image)
        if detection.confidence < cfg.confidence_threshold:
            continue
        key = (detection.class_name, detection.bbox_xyxy_px)
        if key not in unique or detection.confidence > unique[key].confidence:
            unique[key] = detection
    accepted = [(unique[key], localize_detection(unique[key], observation, cfg))
                for key in sorted(unique)]
    edges = []
    for index, (detection, position) in enumerate(accepted):
        if position is None:
            continue
        for track_id, track in tracks.items():
            if track.class_name != detection.class_name:
                continue
            anchor = evidence[track.observation_ids[0]].position_world
            distance = _distance(track.position_world, position)
            if (distance <= cfg.object_match_distance_m
                    and _distance(anchor, position) <= cfg.object_match_distance_m):
                edges.append((distance, track_id, index))
    matched, used = {}, set()
    for _, track_id, index in sorted(edges):
        if index not in matched and track_id not in used:
            matched[index] = track_id
            used.add(track_id)
    for index, (detection, position) in enumerate(accepted):
        track_id = matched.get(index)
        old = tracks.get(track_id)
        if old is None:
            track_counter += 1
            track_id = 'track-%06d' % track_counter
        observation_counter += 1
        observation_id = 'detection-%06d' % observation_counter
        item = DetectionEvidence(observation_id, observation.sequence,
            observation.timestamp_s, track_id, detection.class_name, detection.confidence,
            detection.bbox_xyxy_px, position)
        evidence[observation_id] = item
        ids = old.observation_ids + (observation_id,) if old else (observation_id,)
        samples = [evidence[key] for key in ids]
        if position is not None:
            position = Point3(*(median(getattr(sample.position_world, axis) for sample in samples)
                                for axis in ('x_m', 'y_m', 'z_m')))
        tracks[track_id] = ObjectTrack(track_id, detection.class_name,
            max(sample.confidence for sample in samples), position,
            old.first_seen_s if old else observation.timestamp_s, observation.timestamp_s,
            ids, len({sample.capture_sequence for sample in samples}))
    return dict(sorted(tracks.items())), evidence, track_counter, observation_counter


def _contains(region, position):
    """Continuous polygon containment including edges, independent of grid size."""
    x, y = position.x_m, position.y_m
    vertices = region.boundary_world
    inside = False
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        cross = (x - a.x_m) * (b.y_m - a.y_m) - (y - a.y_m) * (b.x_m - a.x_m)
        if (abs(cross) <= 1e-12
                and min(a.x_m, b.x_m) - 1e-12 <= x <= max(a.x_m, b.x_m) + 1e-12
                and min(a.y_m, b.y_m) - 1e-12 <= y <= max(a.y_m, b.y_m) + 1e-12):
            return True
        if ((a.y_m > y) != (b.y_m > y)
                and x < a.x_m + (y - a.y_m) * (b.x_m - a.x_m) / (b.y_m - a.y_m)):
            inside = not inside
    return inside


def associate_rooms(tracks, regions):
    """Assign only positions contained by exactly one room; ambiguity stays unknown."""
    assignments = {}
    classes = {region.region_id: set() for region in regions}
    for track in tracks.values():
        matches = [region.region_id for region in regions
                   if track.position_world is not None and _contains(region, track.position_world)]
        region_id = matches[0] if len(matches) == 1 else None
        assignments[track.track_id] = region_id
        if region_id is not None:
            classes[region_id].add(track.class_name)
    return assignments, {key: tuple(sorted(value)) for key, value in classes.items()}
