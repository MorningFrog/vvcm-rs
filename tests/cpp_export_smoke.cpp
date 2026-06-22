#include "vvcm_rs.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
    void require(bool condition, const char *message)
    {
        if (!condition)
        {
            std::cerr << message << std::endl;
            std::exit(1);
        }
    }

    bool close_to(float actual, float expected, float tolerance)
    {
        return std::fabs(actual - expected) <= tolerance;
    }

    void require_vec2(const vvcm_rs::Vec2f &actual, const vvcm_rs::Vec2f &expected, float tolerance, const char *message)
    {
        require(close_to(actual.x, expected.x, tolerance) && close_to(actual.y, expected.y, tolerance), message);
    }

    void require_vec3(const vvcm_rs::Vec3f &actual, const vvcm_rs::Vec3f &expected, float tolerance, const char *message)
    {
        require(close_to(actual.x, expected.x, tolerance) && close_to(actual.y, expected.y, tolerance) && close_to(actual.z, expected.z, tolerance), message);
    }

    bool lambda_values_are_stable(const std::vector<float> &values)
    {
        for (float value : values)
        {
            if (!std::isfinite(value) || value < -1.0e-4f)
            {
                return false;
            }
        }
        return true;
    }
} // namespace

int main()
{
    using namespace vvcm_rs;

    require(!version().empty(), "version should not be empty");

    const std::vector<float> formation = {
        213.7f, 122.7f,
        804.6f, 37.2f,
        904.0f, 550.0f,
        439.3f, 715.9f,
    };
    const std::vector<float> sheet = {
        -316.1f, -421.9f,
        803.4f, -384.1f,
        746.1f, 712.8f,
        -367.3f, 664.2f,
    };

    VvcmFk fk(1000.0f, matrix_view(sheet));
    require(fk.robot_count() == 4, "unexpected robot count");
    require(close_to(fk.hold_height(), 1000.0f, 0.001f), "unexpected hold height");

    FkSolutions solutions = fk.update_stable_solutions(matrix_view(formation));
    require(solutions.all_count() == 3, "unexpected solution count");
    require(solutions.stable_count() == 2, "unexpected stable solution count");
    require(fk.solution_count() == 3, "unexpected cached solution count");
    require(fk.stable_solution_count() == 2, "unexpected cached stable solution count");

    const std::vector<FkSolution> stable = solutions.stable();
    require(stable.size() == 2, "unexpected stable vector size");
    require_vec3(stable[0].po, Vec3f{568.8123f, 324.72644f, 336.73608f}, 0.05f, "first stable Po mismatch");
    require_vec2(stable[0].vo, Vec2f{238.6181f, 125.02439f}, 0.05f, "first stable Vo mismatch");
    const std::vector<std::size_t> first_taut = stable[0].taut_cables;
    require(first_taut.size() == 3 && first_taut[0] == 0 && first_taut[1] == 1 && first_taut[2] == 2, "first stable taut cable set mismatch");
    require(stable[0].lambda_values.size() == first_taut.size(), "first stable lambda value count mismatch");
    require(lambda_values_are_stable(stable[0].lambda_values), "first stable lambda values should be finite and non-negative");
    require_vec3(stable[1].po, Vec3f{557.9307f, 341.23087f, 337.2464f}, 0.05f, "second stable Po mismatch");
    require_vec2(stable[1].vo, Vec2f{208.79898f, 152.53357f}, 0.05f, "second stable Vo mismatch");
    const std::vector<std::size_t> second_taut = stable[1].taut_cables;
    require(second_taut.size() == 3 && second_taut[0] == 0 && second_taut[1] == 2 && second_taut[2] == 3, "second stable taut cable set mismatch");
    require(stable[1].lambda_values.size() == second_taut.size(), "second stable lambda value count mismatch");
    require(lambda_values_are_stable(stable[1].lambda_values), "second stable lambda values should be finite and non-negative");
    require(solutions.closest_stable_index(Vec3f{560.0f, 340.0f, 337.0f}) == 1, "closest stable index mismatch");

    try
    {
        FkSolutions().closest_stable_index(Vec3f{0.0f, 0.0f, 0.0f});
        require(false, "empty solution collection should throw");
    }
    catch (const Error &error)
    {
        require(error.code() == VVCM_RS_ERROR_NO_STABLE_SOLUTION, "unexpected no-stable error code");
    }

    VvcmRsFk *raw_fk = nullptr;
    require(vvcm_rs_fk_new(1000.0f, matrix_view(sheet).raw(), &raw_fk) == VVCM_RS_ERROR_OK, "raw C FK construction failed");
    require(vvcm_rs_fk_update_stable_solutions(raw_fk, matrix_view(formation).raw()) == VVCM_RS_ERROR_OK, "raw C FK solve failed");
    VvcmRsFkSolution raw_solution{};
    require(vvcm_rs_fk_solution_at(raw_fk, 0, &raw_solution) == VVCM_RS_ERROR_OK, "raw C solution_at failed");
    require(raw_solution.stable != 0, "raw C first solution should be stable");
    require(raw_solution.taut_cable_count == 3, "raw C taut count mismatch");
    std::size_t raw_taut_count = 0;
    require(vvcm_rs_fk_solution_taut_cables(raw_fk, 0, nullptr, &raw_taut_count) == VVCM_RS_ERROR_OK, "raw C taut count query failed");
    require(raw_taut_count == raw_solution.taut_cable_count, "raw C taut query count mismatch");
    std::vector<std::size_t> raw_taut(raw_taut_count);
    require(vvcm_rs_fk_solution_taut_cables(raw_fk, 0, raw_taut.data(), &raw_taut_count) == VVCM_RS_ERROR_OK, "raw C taut copy failed");
    require(raw_taut.size() == 3 && raw_taut[0] == 0 && raw_taut[1] == 1 && raw_taut[2] == 2, "raw C taut values mismatch");
    std::size_t too_small = 1;
    require(vvcm_rs_fk_solution_taut_cables(raw_fk, 0, raw_taut.data(), &too_small) == VVCM_RS_ERROR_DIMENSION_MISMATCH, "raw C taut copy should reject small buffers");
    require(too_small == 3, "raw C small-buffer response should report required count");
    std::size_t raw_lambda_count = 0;
    require(vvcm_rs_fk_solution_lambda_values(raw_fk, 0, nullptr, &raw_lambda_count) == VVCM_RS_ERROR_OK, "raw C lambda count query failed");
    std::vector<float> raw_lambda(raw_lambda_count);
    require(vvcm_rs_fk_solution_lambda_values(raw_fk, 0, raw_lambda.data(), &raw_lambda_count) == VVCM_RS_ERROR_OK, "raw C lambda copy failed");
    require(lambda_values_are_stable(raw_lambda), "raw C lambda values should be finite and non-negative");
    vvcm_rs_fk_free(raw_fk);

    const std::vector<float> infeasible_sheet = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f,
    };
    const std::vector<float> infeasible_formation = {
        0.0f, 0.0f,
        2.0f, 0.0f,
        2.0f, 2.0f,
        0.0f, 2.0f,
    };
    VvcmFk infeasible_fk(10.0f, matrix_view(infeasible_sheet));
    try
    {
        infeasible_fk.update_stable_solutions(matrix_view(infeasible_formation));
        require(false, "infeasible formation should throw");
    }
    catch (const Error &error)
    {
        require(error.code() == VVCM_RS_ERROR_INFEASIBLE_FORMATION, "unexpected infeasible error code");
    }

    const std::vector<float> six_sheet = {
        -131.665741f, -376.508026f,
        480.675873f, -388.066681f,
        877.700256f, 217.088806f,
        562.778748f, 826.754089f,
        -107.442101f, 918.166626f,
        -453.516937f, 284.887146f,
    };
    const std::vector<float> six_formation = {
        -27.419184f, -176.293854f,
        398.141083f, -35.190411f,
        517.018127f, 338.271301f,
        285.155762f, 609.95575f,
        -175.608231f, 569.463562f,
        -301.437988f, 194.695297f,
    };

    VvcmSimulation simulation(823.0f, matrix_view(six_sheet), matrix_view(six_formation));
    require_vec2(simulation.global_position(), Vec2f{-27.419184f, -176.293854f}, 0.001f, "simulation global position mismatch");
    require_vec2(simulation.formation().row(0), Vec2f{0.0f, 0.0f}, 0.001f, "simulation local origin mismatch");
    require_vec3(simulation.object_position(), Vec3f{137.674f, 420.879f, 301.218f}, 0.2f, "simulation object position mismatch");
    require_vec3(simulation.absolute_object_position(), Vec3f{110.255f, 244.585f, 301.218f}, 0.2f, "simulation absolute object position mismatch");
    require(simulation.has_solution_index(), "simulation should have a selected solution");
    require(simulation.solutions().stable_count() > 0, "simulation cached solutions should expose stable branches");

    const std::vector<float> velocity = {
        5.0f, 5.0f,
        0.0f, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f,
    };
    simulation.set_velocity(matrix_view(velocity));
    simulation.step();
    require_vec2(simulation.global_position(), Vec2f{-27.252517f, -176.12718f}, 0.01f, "simulation global position after step mismatch");
    require_vec2(simulation.formation().row(1), Vec2f{425.394f, 140.937f}, 0.02f, "simulation updated formation mismatch");

    VvcmManualSimulation manual(823.0f, matrix_view(six_sheet));
    require(!manual.has_formation(), "manual simulation should not be initialized yet");
    require_vec2(manual.global_position(), Vec2f{0.0f, 0.0f}, 0.001f, "manual simulation origin mismatch");

    const Vec3f manual_initial = manual.init(matrix_view(six_formation));
    require_vec3(manual_initial, Vec3f{110.255f, 244.585f, 301.218f}, 0.2f, "manual init mismatch");
    require(manual.has_formation(), "manual simulation should now be initialized");
    require(manual.has_object_position(), "manual simulation should now have an object position");
    require_vec2(manual.global_position(), Vec2f{115.97493f, 250.15027f}, 0.01f, "manual centroid mismatch");
    require_vec3(manual.object_position(), Vec3f{-5.71993f, -5.56527f, 301.218f}, 0.2f, "manual local object position mismatch");
    require_vec3(manual.absolute_object_position(), Vec3f{110.255f, 244.585f, 301.218f}, 0.2f, "manual absolute object position mismatch");
    require(manual.has_solution_index(), "manual simulation should have a selected solution");
    require(!manual.taut_cables().empty(), "manual simulation taut cables should not be empty");
    require(manual.solutions().stable_count() > 0, "manual cached solutions should expose stable branches");

    const Vec3f manual_next = manual.get_new_stable_solution(matrix_view(six_formation));
    require_vec3(manual_next, Vec3f{110.255f, 244.585f, 301.218f}, 0.2f, "manual second solve mismatch");

    return 0;
}
