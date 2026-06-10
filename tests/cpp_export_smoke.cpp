#include "vvcm_rs.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
    // Stop immediately with a readable message when a check fails.
    void require(bool condition, const char *message)
    {
        if (!condition)
        {
            std::cerr << message << std::endl;
            std::exit(1);
        }
    }

    // Compare floating-point values with a tolerance because the solver is approximate.
    bool close_to(float actual, float expected, float tolerance)
    {
        return std::fabs(actual - expected) <= tolerance;
    }

    // Compare 2D points component-wise within the requested tolerance.
    void require_point2(const vvcm_rs::Point2 &actual, const vvcm_rs::Point2 &expected, float tolerance, const char *message)
    {
        require(close_to(actual.x, expected.x, tolerance) && close_to(actual.y, expected.y, tolerance), message);
    }

    // Compare 3D points component-wise within the requested tolerance.
    void require_point3(const vvcm_rs::Point3 &actual, const vvcm_rs::Point3 &expected, float tolerance, const char *message)
    {
        require(close_to(actual.x, expected.x, tolerance) && close_to(actual.y, expected.y, tolerance) && close_to(actual.z, expected.z, tolerance), message);
    }
} // namespace

int main()
{
    using namespace vvcm_rs;

    // The exported package should report a usable version string.
    require(!version().empty(), "version should not be empty");

    // Robot endpoints on the world-frame XY plane, using the millimeter-scale README sample.
    const std::vector<Point2> formation = {
        Point2(213.7f, 122.7f),
        Point2(804.6f, 37.2f),
        Point2(904.0f, 550.0f),
        Point2(439.3f, 715.9f),
    };

    // Sheet vertices in the sheet-local XY frame, using the same millimeter-scale sample data.
    const std::vector<Point2> sheet = {
        Point2(-316.1f, -421.9f),
        Point2(803.4f, -384.1f),
        Point2(746.1f, 712.8f),
        Point2(-367.3f, 664.2f),
    };

    // Build the solver and confirm its basic configuration matches the inputs.
    VvcmFk fk(4, 1000.0f, sheet);
    require(fk.robot_count() == 4, "unexpected robot count");
    require(close_to(fk.hold_height(), 1000.0f, 0.001f), "unexpected hold height");

    // Solving once should populate both the full candidate set and the stable subset.
    FkSolutions solutions = fk.update_stable_solutions(formation);
    require(solutions.all_count() == 3, "unexpected solution count");
    require(solutions.stable_count() == 2, "unexpected stable solution count");
    require(fk.solution_count() == 3, "unexpected cached solution count");
    require(fk.stable_solution_count() == 2, "unexpected cached stable solution count");

    // The stable branches should match the known reference poses.
    const std::vector<FkSolution> stable = solutions.stable();
    require(stable.size() == 2, "unexpected stable vector size");
    require_point3(stable[0].po, Point3(568.8123f, 324.72644f, 336.73608f), 0.05f, "first stable Po mismatch");
    require_point2(stable[0].vo, Point2(238.6181f, 125.02439f), 0.05f, "first stable Vo mismatch");
    require(stable[0].taut_cables.size() == 3 && stable[0].taut_cables[0] == 0 && stable[0].taut_cables[1] == 1 && stable[0].taut_cables[2] == 2, "first stable taut cable set mismatch");
    require_point3(stable[1].po, Point3(557.9307f, 341.23087f, 337.2464f), 0.05f, "second stable Po mismatch");
    require_point2(stable[1].vo, Point2(208.79898f, 152.53357f), 0.05f, "second stable Vo mismatch");
    require(stable[1].taut_cables.size() == 3 && stable[1].taut_cables[0] == 0 && stable[1].taut_cables[1] == 2 && stable[1].taut_cables[2] == 3, "second stable taut cable set mismatch");

    // Wrapper-only no-stable errors should also expose a stable error code.
    try
    {
        FkSolutions().closest_stable_to(Point3::zero());
        require(false, "empty solution collection should throw");
    }
    catch (const Error &error)
    {
        require(error.code() == VVCM_RS_ERROR_NO_STABLE_SOLUTION, "unexpected no-stable error code");
    }

    // A stretched formation outside the sheet should preserve the C ABI error code through the C++ wrapper.
    VvcmFk infeasible_fk(4, 10.0f, {
        Point2(0.0f, 0.0f),
        Point2(1.0f, 0.0f),
        Point2(1.0f, 1.0f),
        Point2(0.0f, 1.0f),
    });
    try
    {
        infeasible_fk.update_stable_solutions({
            Point2(0.0f, 0.0f),
            Point2(2.0f, 0.0f),
            Point2(2.0f, 2.0f),
            Point2(0.0f, 2.0f),
        });
        require(false, "infeasible formation should throw");
    }
    catch (const Error &error)
    {
        require(error.code() == VVCM_RS_ERROR_INFEASIBLE_FORMATION, "unexpected infeasible error code");
    }

    // Exercise the velocity-driven simulation wrapper with the shared six-robot fixture.
    VvcmSimulation simulation(6, 823.0f, {
        Point2(-131.665741f, -376.508026f),
        Point2(480.675873f, -388.066681f),
        Point2(877.700256f, 217.088806f),
        Point2(562.778748f, 826.754089f),
        Point2(-107.442101f, 918.166626f),
        Point2(-453.516937f, 284.887146f),
    }, {
        Point2(-27.419184f, -176.293854f),
        Point2(398.141083f, -35.190411f),
        Point2(517.018127f, 338.271301f),
        Point2(285.155762f, 609.95575f),
        Point2(-175.608231f, 569.463562f),
        Point2(-301.437988f, 194.695297f),
    });
    // The initial state should match the documented reference solution.
    require_point2(simulation.global_position(), Point2(-27.419184f, -176.293854f), 0.001f, "simulation global position mismatch");
    require_point2(simulation.formation().front(), Point2::zero(), 0.001f, "simulation local origin mismatch");
    require_point3(simulation.object_position(), Point3(137.674f, 420.879f, 301.218f), 0.2f, "simulation object position mismatch");
    require_point3(simulation.absolute_object_position(), Point3(110.255f, 244.585f, 301.218f), 0.2f, "simulation absolute object position mismatch");
    require(simulation.has_solution_index(), "simulation should have a selected solution");

    simulation.set_velocity({
        Point2(5.0f, 5.0f),
        Point2::zero(),
        Point2::zero(),
        Point2::zero(),
        Point2::zero(),
        Point2::zero(),
    });
    simulation.step();
    // A zero-velocity step should leave the pose unchanged.
    require_point2(simulation.global_position(), Point2(-27.252517f, -176.12718f), 0.01f, "simulation global position after step mismatch");
    require_point2(simulation.formation()[1], Point2(425.394f, 140.937f), 0.02f, "simulation updated formation mismatch");

    // Manual simulation starts from the same sheet but waits for an explicit formation at init.
    VvcmManualSimulation manual(6, 823.0f, {
        Point2(-131.665741f, -376.508026f),
        Point2(480.675873f, -388.066681f),
        Point2(877.700256f, 217.088806f),
        Point2(562.778748f, 826.754089f),
        Point2(-107.442101f, 918.166626f),
        Point2(-453.516937f, 284.887146f),
    });
    require(!manual.has_formation(), "manual simulation should not be initialized yet");
    require_point2(manual.global_position(), Point2::zero(), 0.001f, "manual simulation origin mismatch");

    // Initialize the manual simulation at the world origin and capture the selected branch.
    const Point3 manual_initial = manual.init({
        Point2(-27.419184f, -176.293854f),
        Point2(398.141083f, -35.190411f),
        Point2(517.018127f, 338.271301f),
        Point2(285.155762f, 609.95575f),
        Point2(-175.608231f, 569.463562f),
        Point2(-301.437988f, 194.695297f),
    }, Point3::zero());
    require_point3(manual_initial, Point3(110.255f, 244.585f, 301.218f), 0.2f, "manual init mismatch");
    require(manual.has_formation(), "manual simulation should now be initialized");
    require(manual.has_object_position(), "manual simulation should now have an object position");
    require_point2(manual.global_position(), Point2(115.97493f, 250.15027f), 0.01f, "manual centroid mismatch");
    require_point3(manual.object_position(), Point3(-5.71993f, -5.56527f, 301.218f), 0.2f, "manual local object position mismatch");
    require_point3(manual.absolute_object_position(), Point3(110.255f, 244.585f, 301.218f), 0.2f, "manual absolute object position mismatch");
    require(manual.has_solution_index(), "manual simulation should have a selected solution");
    require(!manual.taut_cables().empty(), "manual simulation taut cables should not be empty");

    // Re-running the same formation should keep the same stable branch.
    const Point3 manual_next = manual.get_new_stable_solution({
        Point2(-27.419184f, -176.293854f),
        Point2(398.141083f, -35.190411f),
        Point2(517.018127f, 338.271301f),
        Point2(285.155762f, 609.95575f),
        Point2(-175.608231f, 569.463562f),
        Point2(-301.437988f, 194.695297f),
    });
    require_point3(manual_next, Point3(110.255f, 244.585f, 301.218f), 0.2f, "manual second solve mismatch");

    return 0;
}
