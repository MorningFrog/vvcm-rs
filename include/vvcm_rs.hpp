#ifndef VVCM_RS_HPP
#define VVCM_RS_HPP

#include "vvcm_rs.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vvcm_rs
{
    class Error : public std::runtime_error
    {
    public:
        Error(VvcmRsErrorCode code, const char *message)
            : std::runtime_error(message == nullptr ? "vvcm-rs error" : message), code_(code) {}

        VvcmRsErrorCode code() const noexcept { return code_; }

    private:
        VvcmRsErrorCode code_;
    };

    inline void check(VvcmRsErrorCode code)
    {
        if (code != VVCM_RS_ERROR_OK)
        {
            throw Error(code, vvcm_rs_last_error_message());
        }
    }

    inline std::string version()
    {
        const char *value = vvcm_rs_version();
        return value == nullptr ? std::string() : std::string(value);
    }

    struct Vec2f
    {
        float x = 0.0f;
        float y = 0.0f;
    };

    struct Vec3f
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct MatrixView2f
    {
        const float *data = nullptr;
        std::size_t rows = 0;
        std::size_t stride = 2;

        MatrixView2f() = default;
        MatrixView2f(const float *data_value, std::size_t row_count, std::size_t row_stride = 2)
            : data(data_value), rows(row_count), stride(row_stride) {}

        explicit MatrixView2f(const std::vector<float> &row_major)
            : data(row_major.empty() ? nullptr : row_major.data()), rows(row_major.size() / 2), stride(2)
        {
            if (row_major.size() % 2 != 0)
            {
                throw std::invalid_argument("MatrixView2f row-major data length must be divisible by 2");
            }
        }

        VvcmRsMat2f raw() const
        {
            return VvcmRsMat2f{data, rows, stride};
        }

        Vec2f row(std::size_t index) const
        {
            if (index >= rows)
            {
                throw std::out_of_range("MatrixView2f row index is out of range");
            }
            const std::size_t offset = index * stride;
            return Vec2f{data[offset], data[offset + 1]};
        }
    };

    inline MatrixView2f matrix_view(const std::vector<float> &row_major)
    {
        return MatrixView2f(row_major);
    }

    struct FkSolution
    {
        bool stable = false;
        Vec3f po{};
        Vec2f vo{};
        std::vector<std::size_t> taut_cables;
        std::vector<float> lambda_values;
    };

    class FkSolutions
    {
    public:
        std::vector<FkSolution> solutions;

        FkSolutions() = default;
        explicit FkSolutions(std::vector<FkSolution> values) : solutions(std::move(values)) {}

        bool is_empty() const { return solutions.empty(); }
        std::size_t all_count() const { return solutions.size(); }

        std::size_t stable_count() const
        {
            std::size_t count = 0;
            for (const FkSolution &solution : solutions)
            {
                if (solution.stable)
                {
                    ++count;
                }
            }
            return count;
        }

        const FkSolution &at(std::size_t index) const
        {
            if (index >= solutions.size())
            {
                throw std::out_of_range("solution index is out of range");
            }
            return solutions[index];
        }

        std::vector<FkSolution> stable() const
        {
            std::vector<FkSolution> out;
            for (const FkSolution &solution : solutions)
            {
                if (solution.stable)
                {
                    out.push_back(solution);
                }
            }
            return out;
        }

        std::size_t closest_stable_index(Vec3f reference) const
        {
            bool found = false;
            std::size_t best_index = 0;
            float best_distance = 0.0f;

            for (std::size_t index = 0; index < solutions.size(); ++index)
            {
                const FkSolution &solution = solutions[index];
                if (!solution.stable)
                {
                    continue;
                }

                const float dx = solution.po.x - reference.x;
                const float dy = solution.po.y - reference.y;
                const float dz = solution.po.z - reference.z;
                const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (!found || distance < best_distance)
                {
                    found = true;
                    best_index = index;
                    best_distance = distance;
                }
            }

            if (!found)
            {
                throw Error(VVCM_RS_ERROR_NO_STABLE_SOLUTION, "no stable VVCM solution found");
            }
            return best_index;
        }
    };

    inline Vec2f vec2_from_raw(VvcmRsPoint2 point)
    {
        return Vec2f{point.x, point.y};
    }

    inline Vec3f vec3_from_raw(VvcmRsPoint3 point)
    {
        return Vec3f{point.x, point.y, point.z};
    }

    template <typename Handle, typename CopyFn>
    std::vector<std::size_t> read_index_values(Handle *handle, std::size_t index, CopyFn copy_fn)
    {
        std::size_t count = 0;
        check(copy_fn(handle, index, nullptr, &count));
        std::vector<std::size_t> values(count);
        std::size_t capacity = count;
        check(copy_fn(handle, index, values.empty() ? nullptr : values.data(), &capacity));
        values.resize(capacity);
        return values;
    }

    template <typename Handle, typename CopyFn>
    std::vector<float> read_float_values(Handle *handle, std::size_t index, CopyFn copy_fn)
    {
        std::size_t count = 0;
        check(copy_fn(handle, index, nullptr, &count));
        std::vector<float> values(count);
        std::size_t capacity = count;
        check(copy_fn(handle, index, values.empty() ? nullptr : values.data(), &capacity));
        values.resize(capacity);
        return values;
    }

    template <typename Handle, typename SolutionFn, typename TautFn, typename LambdaFn>
    FkSolution read_solution(
        Handle *handle,
        std::size_t index,
        SolutionFn solution_fn,
        TautFn taut_fn,
        LambdaFn lambda_fn)
    {
        VvcmRsFkSolution raw{};
        check(solution_fn(handle, index, &raw));
        return FkSolution{
            raw.stable != 0,
            vec3_from_raw(raw.po),
            vec2_from_raw(raw.vo),
            read_index_values(handle, index, taut_fn),
            read_float_values(handle, index, lambda_fn),
        };
    }

    template <typename Handle, typename CountFn, typename SolutionFn, typename TautFn, typename LambdaFn>
    FkSolutions read_solutions(
        Handle *handle,
        CountFn count_fn,
        SolutionFn solution_fn,
        TautFn taut_fn,
        LambdaFn lambda_fn)
    {
        std::size_t count = 0;
        check(count_fn(handle, &count));
        std::vector<FkSolution> values;
        values.reserve(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            values.push_back(read_solution(handle, index, solution_fn, taut_fn, lambda_fn));
        }
        return FkSolutions(std::move(values));
    }

    class VvcmFk
    {
    public:
        VvcmFk(float hold_height, MatrixView2f sheet)
        {
            check(vvcm_rs_fk_new(hold_height, sheet.raw(), &fk_));
        }

        ~VvcmFk()
        {
            vvcm_rs_fk_free(fk_);
        }

        VvcmFk(const VvcmFk &) = delete;
        VvcmFk &operator=(const VvcmFk &) = delete;

        VvcmFk(VvcmFk &&other) noexcept : fk_(std::exchange(other.fk_, nullptr)) {}

        VvcmFk &operator=(VvcmFk &&other) noexcept
        {
            if (this != &other)
            {
                vvcm_rs_fk_free(fk_);
                fk_ = std::exchange(other.fk_, nullptr);
            }
            return *this;
        }

        FkSolutions update_stable_solutions(MatrixView2f formation)
        {
            check(vvcm_rs_fk_update_stable_solutions(fk_, formation.raw()));
            return solutions();
        }

        std::size_t robot_count() const
        {
            std::size_t value = 0;
            check(vvcm_rs_fk_robot_count(fk_, &value));
            return value;
        }

        float hold_height() const
        {
            float value = 0.0f;
            check(vvcm_rs_fk_hold_height(fk_, &value));
            return value;
        }

        std::size_t solution_count() const
        {
            std::size_t value = 0;
            check(vvcm_rs_fk_solution_count(fk_, &value));
            return value;
        }

        std::size_t stable_solution_count() const
        {
            std::size_t value = 0;
            check(vvcm_rs_fk_stable_solution_count(fk_, &value));
            return value;
        }

        FkSolutions solutions()
        {
            return read_solutions(
                fk_,
                vvcm_rs_fk_solution_count,
                vvcm_rs_fk_solution_at,
                vvcm_rs_fk_solution_taut_cables,
                vvcm_rs_fk_solution_lambda_values);
        }

    private:
        VvcmRsFk *fk_ = nullptr;
    };

    class VvcmSimulation
    {
    public:
        VvcmSimulation(
            float hold_height,
            MatrixView2f sheet,
            MatrixView2f initial_formation,
            Vec3f po_initial = Vec3f{},
            float dt = 0.033333335f)
        {
            const float po[3] = {po_initial.x, po_initial.y, po_initial.z};
            check(vvcm_rs_simulation_new(
                hold_height,
                sheet.raw(),
                initial_formation.raw(),
                po,
                dt,
                &simulation_));
        }

        ~VvcmSimulation()
        {
            vvcm_rs_simulation_free(simulation_);
        }

        VvcmSimulation(const VvcmSimulation &) = delete;
        VvcmSimulation &operator=(const VvcmSimulation &) = delete;

        void set_velocity(MatrixView2f velocity)
        {
            check(vvcm_rs_simulation_set_velocity(simulation_, velocity.raw()));
        }

        void step()
        {
            check(vvcm_rs_simulation_step(simulation_));
        }

        Vec2f global_position() const
        {
            float point[2] = {};
            check(vvcm_rs_simulation_global_position(simulation_, point));
            return Vec2f{point[0], point[1]};
        }

        Vec3f object_position() const
        {
            float point[3] = {};
            check(vvcm_rs_simulation_object_position(simulation_, point));
            return Vec3f{point[0], point[1], point[2]};
        }

        Vec3f absolute_object_position() const
        {
            float point[3] = {};
            check(vvcm_rs_simulation_absolute_object_position(simulation_, point));
            return Vec3f{point[0], point[1], point[2]};
        }

        bool has_solution_index() const
        {
            uint8_t has_value = 0;
            std::size_t index = 0;
            check(vvcm_rs_simulation_solution_index(simulation_, &has_value, &index));
            return has_value != 0;
        }

        MatrixView2f formation()
        {
            VvcmRsMat2f view{};
            check(vvcm_rs_simulation_formation_view(simulation_, &view));
            return MatrixView2f(view.data, view.rows, view.stride);
        }

        MatrixView2f absolute_formation()
        {
            VvcmRsMat2f view{};
            check(vvcm_rs_simulation_absolute_formation_view(simulation_, &view));
            return MatrixView2f(view.data, view.rows, view.stride);
        }

        MatrixView2f velocity()
        {
            VvcmRsMat2f view{};
            check(vvcm_rs_simulation_velocity_view(simulation_, &view));
            return MatrixView2f(view.data, view.rows, view.stride);
        }

        FkSolutions solutions()
        {
            return read_solutions(
                simulation_,
                vvcm_rs_simulation_solution_count,
                vvcm_rs_simulation_solution_at,
                vvcm_rs_simulation_solution_taut_cables,
                vvcm_rs_simulation_solution_lambda_values);
        }

    private:
        VvcmRsSimulation *simulation_ = nullptr;
    };

    class VvcmManualSimulation
    {
    public:
        VvcmManualSimulation(float hold_height, MatrixView2f sheet)
        {
            check(vvcm_rs_manual_simulation_new(hold_height, sheet.raw(), &simulation_));
        }

        ~VvcmManualSimulation()
        {
            vvcm_rs_manual_simulation_free(simulation_);
        }

        VvcmManualSimulation(const VvcmManualSimulation &) = delete;
        VvcmManualSimulation &operator=(const VvcmManualSimulation &) = delete;

        Vec3f init(MatrixView2f formation, Vec3f po_initial = Vec3f{})
        {
            const float po[3] = {po_initial.x, po_initial.y, po_initial.z};
            float out[3] = {};
            check(vvcm_rs_manual_simulation_init(simulation_, formation.raw(), po, out));
            return Vec3f{out[0], out[1], out[2]};
        }

        Vec3f get_new_stable_solution(MatrixView2f formation)
        {
            float out[3] = {};
            check(vvcm_rs_manual_simulation_get_new_stable_solution(
                simulation_,
                formation.raw(),
                out));
            return Vec3f{out[0], out[1], out[2]};
        }

        Vec2f global_position() const
        {
            float point[2] = {};
            check(vvcm_rs_manual_simulation_global_position(simulation_, point));
            return Vec2f{point[0], point[1]};
        }

        bool has_formation() const
        {
            uint8_t has_value = 0;
            check(vvcm_rs_manual_simulation_has_formation(simulation_, &has_value));
            return has_value != 0;
        }

        MatrixView2f formation()
        {
            VvcmRsMat2f view{};
            check(vvcm_rs_manual_simulation_formation_view(simulation_, &view));
            return MatrixView2f(view.data, view.rows, view.stride);
        }

        bool has_object_position() const
        {
            uint8_t has_value = 0;
            float point[3] = {};
            check(vvcm_rs_manual_simulation_object_position(simulation_, &has_value, point));
            return has_value != 0;
        }

        Vec3f object_position() const
        {
            uint8_t has_value = 0;
            float point[3] = {};
            check(vvcm_rs_manual_simulation_object_position(simulation_, &has_value, point));
            if (has_value == 0)
            {
                throw Error(VVCM_RS_ERROR_INVALID_ARGUMENT, "manual simulation has no object position");
            }
            return Vec3f{point[0], point[1], point[2]};
        }

        Vec3f absolute_object_position() const
        {
            uint8_t has_value = 0;
            float point[3] = {};
            check(vvcm_rs_manual_simulation_absolute_object_position(simulation_, &has_value, point));
            if (has_value == 0)
            {
                throw Error(VVCM_RS_ERROR_INVALID_ARGUMENT, "manual simulation has no object position");
            }
            return Vec3f{point[0], point[1], point[2]};
        }

        bool has_solution_index() const
        {
            uint8_t has_value = 0;
            std::size_t index = 0;
            check(vvcm_rs_manual_simulation_solution_index(simulation_, &has_value, &index));
            return has_value != 0;
        }

        std::vector<std::size_t> taut_cables() const
        {
            std::size_t count = 0;
            check(vvcm_rs_manual_simulation_taut_cable_count(simulation_, &count));
            std::vector<std::size_t> out(count);
            for (std::size_t index = 0; index < count; ++index)
            {
                check(vvcm_rs_manual_simulation_taut_cable_at(simulation_, index, &out[index]));
            }
            return out;
        }

        FkSolutions solutions()
        {
            return read_solutions(
                simulation_,
                vvcm_rs_manual_simulation_solution_count,
                vvcm_rs_manual_simulation_solution_at,
                vvcm_rs_manual_simulation_solution_taut_cables,
                vvcm_rs_manual_simulation_solution_lambda_values);
        }

    private:
        VvcmRsManualSimulation *simulation_ = nullptr;
    };
} // namespace vvcm_rs

#endif
