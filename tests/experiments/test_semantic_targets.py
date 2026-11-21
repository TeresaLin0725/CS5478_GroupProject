import math

import pytest

from frontier_search.evaluation.retained import RetainedFrames
from frontier_search.runner import integrated
from frontier_search.simulation.igibson_adapter import AdapterConfig
from frontier_search.simulation.integrated_rs import (
    CloseRetainedEvaluator, RsBackend, RsTargetSpec,
)


def test_rs_target_spec_is_validated_and_bound_to_backend_and_evaluator():
    target = RsTargetSpec('banana', '011_banana', (-1.4, 2.95, .897), .5,
                          'Rs-semantic-banana')
    backend = RsBackend(AdapterConfig(), 'apartment', target)
    evaluator = CloseRetainedEvaluator(RetainedFrames(), object(), target_class='banana')
    assert backend.target is target
    assert evaluator._target_class == 'banana'


@pytest.mark.parametrize('target', (
    RsTargetSpec('', '025_mug', (0., 0., 0.)),
    RsTargetSpec('cup', '', (0., 0., 0.)),
    RsTargetSpec('cup', '025_mug', (0., 0.)),
    RsTargetSpec('cup', '025_mug', (0., 0., math.inf)),
    RsTargetSpec('cup', '025_mug', (0., 0., 0.), math.nan),
    RsTargetSpec('cup', '025_mug', (0., 0., 0.), 0., 'target', math.nan),
))
def test_invalid_rs_target_specs_are_rejected(target):
    with pytest.raises(ValueError):
        target.validate()


def test_runtime_configuration_records_target_setup():
    manifest = integrated.shared_round_manifest_for('semantic-target')
    target = RsTargetSpec('scissors', '037_scissors', (.15, -3.8, .745), -.63,
                          'Rs-semantic-scissors')
    configuration = integrated.runtime_configuration(manifest, 'apartment', None, target)
    assert configuration['target']['class_name'] == 'scissors'
    assert configuration['target']['model'] == '037_scissors'
    assert configuration['target']['position'] == (.15, -3.8, .745)
    assert configuration['target']['yaw_rad'] == -.63
