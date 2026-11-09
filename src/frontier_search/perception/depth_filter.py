"""Optional range-gated median depth filter for geometric map evidence only.

Only measured valid centres with a majority of similar-depth neighbours are
replaced. Missing pixels are not filled; isolated/thin surfaces without enough
support retain their original measurement. No simulator truth or noise settings
are consulted. Like all spatial filters, this is an approximation, not a proof
that arbitrary sub-pixel or low-contrast obstacles will be preserved.
"""
from dataclasses import replace


def filter_mapping_depth(observation, config):
    size = config.depth_filter_kernel_size
    if not size:
        return observation
    import numpy as np
    image = observation.depth_image
    depth = np.frombuffer(image.data, dtype='<f4').reshape(image.height, image.width)
    valid = np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
    radius = size // 2
    padded = np.pad(depth, radius, mode='constant', constant_values=np.nan)
    # Missing border pixels do not count as repeated support. Work in row blocks
    # to keep the median's temporary arrays bounded at native camera resolution.
    output = depth.copy()
    # Fusion projects only stride-aligned centres. Their neighbours still use
    # full-resolution pixels; avoid computing medians for unused ray endpoints.
    stride = config.depth_pixel_stride
    for first in range(0, image.height, 32 * stride):
        last = min(first + 32 * stride, image.height)
        centre = depth[first:last:stride, ::stride]
        centre_valid = valid[first:last:stride, ::stride]
        neighbours = []
        for dy in range(size):
            for dx in range(size):
                values = padded[first + dy:last + dy:stride, dx:dx + image.width:stride]
                with np.errstate(invalid='ignore'):
                    keep = (centre_valid & np.isfinite(values)
                            & (values >= config.depth_min_m) & (values <= config.depth_max_m)
                            & (np.abs(values - centre) <= config.depth_filter_max_delta_m))
                neighbours.append(np.where(keep, values, np.inf))
        samples = np.stack(neighbours, axis=0)
        count = np.sum(np.isfinite(samples), axis=0)
        enough = centre_valid & (count >= size * size // 2 + 1)
        if not np.any(enough):
            continue
        samples.sort(axis=0)
        # Use the actual median of accepted measurements, including even counts.
        lower = np.take_along_axis(samples, np.maximum((count - 1) // 2, 0)[None], axis=0)[0]
        upper = np.take_along_axis(samples, (count // 2)[None], axis=0)[0]
        filtered = .5 * lower + .5 * upper
        output[first:last:stride, ::stride][enough] = filtered[enough]
    if output.tobytes() == image.data:
        return observation
    return replace(observation, depth_image=replace(image, data=output.tobytes()))
