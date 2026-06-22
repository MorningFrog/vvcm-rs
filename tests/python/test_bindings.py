import numpy as np
import pytest

from vvcm_rs import (
    DimensionMismatchError,
    FkSolution,
    FkSolutions,
    InfeasibleFormationError,
    NoSolutionError,
    NoStableSolutionError,
    VVCM_FK,
    VvcmError,
    VvcmFk,
    VvcmManualSimulation,
    VvcmSimulation,
)


def test_fk_sample_accepts_numpy_arrays_and_returns_stable_solutions():
    fk = VvcmFk(1000.0, readme_sheet_array())

    solutions = fk.update_stable_solutions(readme_formation_array())

    assert isinstance(solutions, FkSolutions)
    assert solutions.all_count() == 3
    assert solutions.stable_count() == 2
    assert len(solutions.solutions) == 3
    assert [solution.stable for solution in solutions.solutions] == [True, True, False]

    stable_solutions = solutions.stable()
    assert len(stable_solutions) == 2
    assert all(isinstance(solution, FkSolution) for solution in stable_solutions)

    first = stable_solutions[0]
    assert_point3_close(first.po, np.array([568.8123, 324.72644, 336.73608], dtype=np.float32), 0.05)
    assert_point2_close(first.vo, np.array([238.6181, 125.02439], dtype=np.float32), 0.05)
    assert first.taut_cables.tolist() == [0, 1, 2]
    assert len(first.lambda_values) == len(first.taut_cables)
    assert np.all(np.isfinite(first.lambda_values))
    assert np.all(first.lambda_values >= -1.0e-4)

    second = stable_solutions[1]
    assert_point3_close(second.po, np.array([557.9307, 341.23087, 337.2464], dtype=np.float32), 0.05)
    assert_point2_close(second.vo, np.array([208.79898, 152.53357], dtype=np.float32), 0.05)
    assert second.taut_cables.tolist() == [0, 2, 3]
    assert np.all(second.lambda_values >= -1.0e-4)

    assert solutions.closest_stable_to(np.array([560.0, 340.0, 337.0], dtype=np.float32)) == 1


def test_empty_solution_collection_exposes_solution_list():
    solutions = FkSolutions()

    assert solutions.is_empty()
    assert solutions.all_count() == 0
    assert solutions.stable_count() == 0
    assert solutions.solutions == []
    assert solutions.stable() == []
    assert solutions.closest_stable_to(np.zeros(3, dtype=np.float32)) is None


def test_aliases_match_cpp_style_class_names():
    assert VVCM_FK is VvcmFk


def test_errors_are_mapped_to_python_exception():
    fk = VvcmFk(1000.0, readme_sheet_array())

    with pytest.raises(DimensionMismatchError, match="dimension mismatch") as caught:
        fk.update_stable_solutions(
            np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=np.float32)
        )
    assert isinstance(caught.value, VvcmError)


def test_invalid_numpy_shapes_are_rejected():
    with pytest.raises(ValueError, match="sheet must have shape"):
        VvcmFk(1000.0, np.zeros((4, 3), dtype=np.float32))

    fk = VvcmFk(1000.0, readme_sheet_array())
    with pytest.raises(TypeError, match="formation must be C-contiguous"):
        fk.update_stable_solutions(readme_formation_array()[::2])


def test_infeasible_formation_maps_to_python_exception_subclass():
    fk = VvcmFk(
        10.0,
        np.array(
            [
                [0.0, 0.0],
                [1.0, 0.0],
                [1.0, 1.0],
                [0.0, 1.0],
            ],
            dtype=np.float32,
        ),
    )

    with pytest.raises(InfeasibleFormationError, match="robot formation is infeasible") as caught:
        fk.update_stable_solutions(
            np.array(
                [
                    [0.0, 0.0],
                    [2.0, 0.0],
                    [2.0, 2.0],
                    [0.0, 2.0],
                ],
                dtype=np.float32,
            )
        )
    assert isinstance(caught.value, VvcmError)


def test_all_typed_error_classes_derive_from_vvcm_error():
    assert issubclass(DimensionMismatchError, VvcmError)
    assert issubclass(InfeasibleFormationError, VvcmError)
    assert issubclass(NoSolutionError, VvcmError)
    assert issubclass(NoStableSolutionError, VvcmError)


def test_manual_simulation_returns_expected_branch():
    simulation = VvcmManualSimulation(823.0, six_robot_sheet())

    po = simulation.init(six_robot_formation(), np.zeros(3, dtype=np.float32))

    assert_point3_close(po, np.array([110.255, 244.585, 301.218], dtype=np.float32), 0.2)
    assert_point3_close(
        simulation.absolute_object_position,
        np.array([110.255, 244.585, 301.218], dtype=np.float32),
        0.2,
    )
    assert simulation.solution_index is not None
    assert simulation.taut_cables.size > 0
    assert simulation.formation.shape == (6, 2)

    po = simulation.get_new_stable_solution(six_robot_formation())
    assert_point3_close(po, np.array([110.255, 244.585, 301.218], dtype=np.float32), 0.2)


def test_velocity_simulation_steps_consistently():
    simulation = VvcmSimulation(
        823.0,
        six_robot_sheet(),
        six_robot_formation(),
        np.zeros(3, dtype=np.float32),
        1.0 / 30.0,
    )

    assert_point2_close(simulation.global_position, np.array([-27.419184, -176.293854], dtype=np.float32), 0.001)
    assert_point2_close(simulation.formation[0], np.array([0.0, 0.0], dtype=np.float32), 0.001)
    assert_point3_close(simulation.object_position, np.array([137.674, 420.879, 301.218], dtype=np.float32), 0.2)
    assert_point3_close(
        simulation.absolute_object_position(),
        np.array([110.255, 244.585, 301.218], dtype=np.float32),
        0.2,
    )

    before_zero_step = simulation.object_position.copy()
    simulation.step()
    np.testing.assert_array_equal(simulation.object_position, before_zero_step)

    simulation.set_velocity(
        np.array(
            [
                [5.0, 5.0],
                [0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
                [0.0, 0.0],
            ],
            dtype=np.float32,
        )
    )
    simulation.step()

    assert_point2_close(simulation.global_position, np.array([-27.252517, -176.12718], dtype=np.float32), 0.01)
    assert_point2_close(simulation.formation[0], np.array([0.0, 0.0], dtype=np.float32), 0.001)
    assert_point2_close(simulation.formation[1], np.array([425.394, 140.937], dtype=np.float32), 0.02)
    assert_point3_close(simulation.object_position, np.array([137.54, 420.572, 301.209], dtype=np.float32), 0.25)


def readme_formation_array():
    return np.array(
        [
            [213.7, 122.7],
            [804.6, 37.2],
            [904.0, 550.0],
            [439.3, 715.9],
        ],
        dtype=np.float32,
    )


def readme_sheet_array():
    return np.array(
        [
            [-316.1, -421.9],
            [803.4, -384.1],
            [746.1, 712.8],
            [-367.3, 664.2],
        ],
        dtype=np.float32,
    )


def six_robot_formation():
    return np.array(
        [
            [-27.419184, -176.293854],
            [398.141083, -35.190411],
            [517.018127, 338.271301],
            [285.155762, 609.95575],
            [-175.608231, 569.463562],
            [-301.437988, 194.695297],
        ],
        dtype=np.float32,
    )


def six_robot_sheet():
    return np.array(
        [
            [-131.665741, -376.508026],
            [480.675873, -388.066681],
            [877.700256, 217.088806],
            [562.778748, 826.754089],
            [-107.442101, 918.166626],
            [-453.516937, 284.887146],
        ],
        dtype=np.float32,
    )


def assert_point2_close(actual, expected, tolerance):
    np.testing.assert_allclose(actual[:2], expected[:2], atol=tolerance, rtol=0.0)


def assert_point3_close(actual, expected, tolerance):
    np.testing.assert_allclose(actual[:3], expected[:3], atol=tolerance, rtol=0.0)
