"""Load allowed room geometry without importing occupancy or object ground truth."""

import json
from pathlib import Path
from typing import Tuple

from ..contracts import Point2, RegionPrior
from ..validation import ContractError, is_finite_number, require



# Check room geometry and enforce the episode's semantic-label policy.
def validate_region_priors(priors: Tuple[RegionPrior, ...],
                           oracle_room_labels: bool = False) -> None:
    require(type(oracle_room_labels) is bool, "oracle_room_labels must be boolean")
    require(isinstance(priors, tuple), "region priors must be a tuple")
    region_ids = set()
    for region in priors:
        require(isinstance(region, RegionPrior), "priors require RegionPrior records")
        require(isinstance(region.region_id, str) and bool(region.region_id.strip()), "missing region ID")
        require(region.region_id not in region_ids, "duplicate region ID")
        region_ids.add(region.region_id)
        require(oracle_room_labels or region.room_type is None,
                "standard initialization contains oracle room labels")
        require(region.room_type is None or isinstance(region.room_type, str)
                and bool(region.room_type.strip()), "room label must be nonempty text or None")
        vertices = region.boundary_world
        require(isinstance(vertices, tuple), "room vertices must be a tuple")
        require(all(isinstance(p, Point2) and is_finite_number(p.x_m)
                    and is_finite_number(p.y_m) for p in vertices),
                "room polygon requires finite Point2 coordinates")
        require(len({(p.x_m, p.y_m) for p in vertices}) >= 3,
                "room polygon requires at least three distinct vertices")
        origin = vertices[0]
        try:
            points = tuple((p.x_m - origin.x_m, p.y_m - origin.y_m) for p in vertices)
            twice_area = sum(a[0] * b[1] - b[0] * a[1]
                             for a, b in zip(points, points[1:] + points[:1]))
        except OverflowError as exc:
            raise ContractError("room polygon area overflow") from exc
        require(is_finite_number(twice_area) and twice_area != 0,
                "room polygon must have finite nonzero area")


# Load immutable world-space room polygons from an explicit prior file.
def load_region_priors(path: Path, oracle_room_labels: bool = False) -> Tuple[RegionPrior, ...]:
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise ContractError("cannot read room priors: %s" % exc) from exc
    require(isinstance(document, dict) and set(document) == {"schema_version", "frame_id", "regions"},
            "prior file requires only schema_version, frame_id and regions")
    require(document["schema_version"] == "1.0", "unsupported room-prior schema")
    require(document["frame_id"] == "world", "room priors must use world coordinates in metres")
    require(isinstance(document["regions"], list), "regions must be a list")
    priors = []
    for region in document["regions"]:
        require(isinstance(region, dict) and {"region_id", "boundary_world"} <= set(region)
                and set(region) <= {"region_id", "boundary_world", "room_type"},
                "region requires region_id, boundary_world and optional room_type only")
        require(isinstance(region["region_id"], str) and bool(region["region_id"].strip()),
                "missing region ID")
        label = region.get("room_type")
        require(label is None or isinstance(label, str) and bool(label.strip()),
                "room label must be nonempty text or None")
        vertices = region["boundary_world"]
        require(isinstance(vertices, list) and all(isinstance(p, list) and len(p) == 2
                and all(is_finite_number(x) for x in p) for p in vertices),
                "boundary_world must contain finite [x_m, y_m] vertices")
        priors.append(RegionPrior(region["region_id"], tuple(Point2(*p) for p in vertices),
                                  region.get("room_type")))
    result = tuple(priors)
    validate_region_priors(result, oracle_room_labels)
    return result
