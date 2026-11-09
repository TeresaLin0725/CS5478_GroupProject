"""Depth perturbations at the runner's perception boundary, after truth retention.

Streams are keyed by seed and capture sequence, never policy or episode name.
Gaussian noise and dropout have independent streams for controlled ablations.
No global RNG is read or changed. Disabled noise returns the original observation.
"""
from dataclasses import asdict, replace
import json
from pathlib import Path

from ..config import SensorNoiseConfig


class DepthNoise:
    def __init__(self, config, mapping):
        config.__post_init__()
        self.config = config
        self.minimum = mapping.depth_min_m
        self.maximum = mapping.depth_max_m
        self.stats = dict(captures=0, valid_pixels=0, dropout_pixels=0,
                          out_of_range_pixels=0, modified_pixels=0)

    def apply(self, observation):
        if not self.config.enabled:
            return observation
        # Keep the dependency-free clean/synthetic path intact.
        import numpy as np
        cfg = self.config
        image = observation.depth_image
        depth = np.frombuffer(image.data, dtype='<f4').reshape(image.height, image.width)
        valid = np.isfinite(depth) & (depth >= self.minimum) & (depth <= self.maximum)
        output = depth.copy()
        dropped = np.zeros(depth.shape, dtype=bool)
        outside = np.zeros(depth.shape, dtype=bool)
        if cfg.depth_std_m:
            rng = np.random.Generator(np.random.PCG64(
                np.random.SeedSequence([cfg.seed, observation.sequence, 0])))
            # Float64 arithmetic avoids overflow for large, but finite, stress levels.
            perturbed = depth.astype(np.float64)
            perturbed[valid] += rng.normal(0., cfg.depth_std_m, depth.shape)[valid]
            outside = valid & (~np.isfinite(perturbed) | (perturbed < self.minimum)
                               | (perturbed > self.maximum))
            kept = valid & ~outside
            output[kept] = perturbed[kept]
            output[outside] = 0.
        if cfg.depth_dropout_prob:
            rng = np.random.Generator(np.random.PCG64(
                np.random.SeedSequence([cfg.seed, observation.sequence, 1])))
            dropped = valid & (rng.random(depth.shape) < cfg.depth_dropout_prob)
            output[dropped] = 0.
        modified = valid & (output != depth)
        self.stats['captures'] += 1
        self.stats['valid_pixels'] += int(np.count_nonzero(valid))
        self.stats['dropout_pixels'] += int(np.count_nonzero(dropped))
        self.stats['out_of_range_pixels'] += int(np.count_nonzero(outside & ~dropped))
        self.stats['modified_pixels'] += int(np.count_nonzero(modified))
        if not np.any(modified):
            return observation
        return replace(observation, depth_image=replace(image, data=output.astype('<f4').tobytes()))


def add_noise_arguments(parser):
    group = parser.add_argument_group('sensor noise (off by default)')
    switch = group.add_mutually_exclusive_group()
    switch.add_argument('--noise', dest='noise_enabled', action='store_true', default=None,
                        help='enable depth noise and dropout with the configured settings')
    switch.add_argument('--no-noise', dest='noise_enabled', action='store_false', default=None,
                        help='disable noise, including noise enabled by a JSON config')
    group.add_argument('--noise-config', type=Path, help='JSON SensorNoiseConfig; CLI fields override it')
    group.add_argument('--depth-noise-std-m', type=float, help='Gaussian depth standard deviation in metres (default .01)')
    group.add_argument('--depth-dropout-prob', type=float, help='valid-pixel dropout probability (default .01)')
    group.add_argument('--noise-seed', type=int, help='dedicated noise seed (default 0)')


def noise_arguments_supplied(args):
    return any(getattr(args, key, None) is not None for key in (
        'noise_enabled', 'noise_config', 'depth_noise_std_m', 'depth_dropout_prob', 'noise_seed'))


def noise_from_arguments(args):
    values = asdict(SensorNoiseConfig())
    if args.noise_config is not None:
        data = json.loads(args.noise_config.read_text(encoding='utf-8-sig'))
        if not isinstance(data, dict) or set(data) - set(values):
            raise ValueError('noise config must be an object containing only: ' + ', '.join(values))
        values.update(data)
    for source, target in (('noise_enabled', 'enabled'), ('depth_noise_std_m', 'depth_std_m'),
                           ('depth_dropout_prob', 'depth_dropout_prob'), ('noise_seed', 'seed')):
        value = getattr(args, source)
        if value is not None:
            values[target] = value
    return SensorNoiseConfig(**values)
