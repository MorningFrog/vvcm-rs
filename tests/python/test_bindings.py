import math

import numpy as np
import pytest

from vvcm_rs import (
    Point2,
    Point3,
    RobotFormation,
    SheetShape,
    VVCM_FK,
    VvcmError,
    VvcmFk,
    VvcmManualSimulation,
    VvcmSimulation,
)


def test_fk_sample_accepts_numpy_arrays_and_returns_stable_solutions():
    # NumPy arrays should flow through the Python wrapper without extra conversion helpers.
    fk = VvcmFk(4, 1000.0, readme_sheet_array())

    # Solve the README sample and inspect the stable branches it returns.
    solutions = fk.update_stable_solutions(readme_formation_array())

    assert solutions.all_count() == 3
    assert solutions.stable_count() == 2
    # The known stable branches should match the reference pose values.
    stable = solutions.stable()
    assert_point3_close(stable[0].po, Point3(568.8123, 324.72644, 336.73608), 0.05)
    assert_point2_close(stable[0].vo, Point2(238.6181, 125.02439), 0.05)
    assert stable[0].taut_cables == [0, 1, 2]
    assert_point3_close(stable[1].po, Point3(557.9307, 341.23087, 337.2464), 0.05)
    assert_point2_close(stable[1].vo, Point2(208.79898, 152.53357), 0.05)
    assert stable[1].taut_cables == [0, 2, 3]

    # The nearest-stable helper should pick the branch closest to the query point.
    closest_index, closest = solutions.closest_stable_to((560.0, 340.0, 337.0))
    assert closest_index == 1
    assert closest.stable is True


def test_domain_types_accept_lists_and_point_instances():
    # The Python API should accept both Point2 instances and plain tuple rows.
    formation = RobotFormation([Point2(1.0, 2.0), (3.0, 4.0)])

    assert len(formation) == 2
    assert formation[0].as_tuple() == (1.0, 2.0)
    assert formation.as_tuples() == [(1.0, 2.0), (3.0, 4.0)]
    assert_point2_close(formation.centroid(), Point2(2.0, 3.0), 1.0e-6)

    relative = formation.relative_to(Point2(1.0, 1.0))
    assert relative.as_tuples() == [(0.0, 1.0), (2.0, 3.0)]

    sheet = SheetShape([Point2(0.0, 0.0), Point2(1.0, 0.0), Point2(0.0, 1.0)])
    assert len(sheet) == 3
    assert sheet.vertices[2].as_tuple() == (0.0, 1.0)


def test_aliases_match_cpp_style_class_names():
    # Keep the C++-style alias for compatibility with downstream code.
    assert VVCM_FK is VvcmFk


def test_errors_are_mapped_to_python_exception():
    # Dimension mismatches should raise the package-specific exception class.
    fk = VvcmFk(4, 1000.0, readme_sheet_array())

    with pytest.raises(VvcmError, match="dimension mismatch"):
        fk.update_stable_solutions([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)])


def test_manual_simulation_returns_expected_branch():
    # Manual simulation mirrors the Rust smoke test on the shared six-robot fixture.
    simulation = VvcmManualSimulation(6, 823.0, six_robot_sheet())

    # Initialize at the world origin to select the same stable branch as the Rust test.
    po = simulation.init(six_robot_formation(), Point3.zero())

    assert_point3_close(po, Point3(110.255, 244.585, 301.218), 0.2)
    assert_point3_close(
        simulation.absolute_object_position,
        Point3(110.255, 244.585, 301.218),
        0.2,
    )
    assert simulation.solution_index is not None
    assert len(simulation.taut_cables) > 0

    # Re-solving the same formation should keep the same stable branch.
    po = simulation.get_new_stable_solution(six_robot_formation())
    assert_point3_close(po, Point3(110.255, 244.585, 301.218), 0.2)


def test_velocity_simulation_steps_consistently():
    # The velocity-driven wrapper should preserve the initial branch and step consistently.
    simulation = VvcmSimulation(
        6,
        823.0,
        six_robot_sheet(),
        six_robot_formation(),
        Point3.zero(),
        1.0 / 30.0,
    )

    # The local formation is expressed relative to the first robot, so the first point becomes the origin.
    assert_point2_close(simulation.global_position, Point2(-27.419184, -176.293854), 0.001)
    assert_point2_close(simulation.formation[0], Point2.zero(), 0.001)
    assert_point3_close(simulation.object_position, Point3(137.674, 420.879, 301.218), 0.2)
    assert_point3_close(
        simulation.absolute_object_position(),
        Point3(110.255, 244.585, 301.218),
        0.2,
    )

    before_zero_step = simulation.object_position.as_tuple()
    # A zero-velocity step should not move the object.
    simulation.step()
    assert simulation.object_position.as_tuple() == before_zero_step

    # Apply a small velocity to the first robot and step once more.
    simulation.set_velocity(
        [
            (5.0, 5.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ]
    )
    simulation.step()

    # Confirm the frame and object pose advance consistently after the update.
    assert_point2_close(simulation.global_position, Point2(-27.252517, -176.12718), 0.01)
    assert_point2_close(simulation.formation[0], Point2.zero(), 0.001)
    assert_point2_close(simulation.formation[1], Point2(425.394, 140.937), 0.02)
    assert_point3_close(simulation.object_position, Point3(137.54, 420.572, 301.209), 0.25)


def readme_formation_array():
    # Keep this fixture identical to the README usage snippet; the robot endpoints live on the world-frame XY plane.
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
    # Keep this fixture identical to the README usage snippet; the sheet vertices live in the sheet-local XY frame.
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
    # Shared six-robot fixture for robot endpoints on the world-frame XY plane.
    return [
        (-27.419184, -176.293854),
        (398.141083, -35.190411),
        (517.018127, 338.271301),
        (285.155762, 609.95575),
        (-175.608231, 569.463562),
        (-301.437988, 194.695297),
    ]


def six_robot_sheet():
    # Matching sheet-local XY fixture for the shared six-robot simulation case.
    return [
        (-131.665741, -376.508026),
        (480.675873, -388.066681),
        (877.700256, 217.088806),
        (562.778748, 826.754089),
        (-107.442101, 918.166626),
        (-453.516937, 284.887146),
    ]


def assert_point2_close(actual, expected, tolerance):
    # Compare each axis with a tolerance because the Python API uses floating-point values.
    assert math.isclose(actual.x, expected.x, abs_tol=tolerance), (
        actual.x,
        expected.x,
    )
    assert math.isclose(actual.y, expected.y, abs_tol=tolerance), (
        actual.y,
        expected.y,
    )


def assert_point3_close(actual, expected, tolerance):
    # Reuse the 2D helper for x/y and compare z separately.
    assert_point2_close(actual, expected, tolerance)
    assert math.isclose(actual.z, expected.z, abs_tol=tolerance), (
        actual.z,
        expected.z,
    )
