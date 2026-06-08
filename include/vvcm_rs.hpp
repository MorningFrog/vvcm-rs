#ifndef VVCM_RS_HPP
#define VVCM_RS_HPP

#include "vvcm_rs.h"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vvcm_rs
{
    class Error : public std::runtime_error
    {
    public:
        explicit Error(const std::string &message) : std::runtime_error(message) {}
    };

    inline void throw_on_error(VvcmRsErrorCode code)
    {
        if (code == VVCM_RS_ERROR_OK)
        {
            return;
        }

        const char *message = vvcm_rs_last_error_message();
        if (message == nullptr || message[0] == '\0')
        {
            message = vvcm_rs_error_message(code);
        }

        throw Error(message == nullptr ? "vvcm-rs error" : message);
    }

    inline std::string version()
    {
        const char *value = vvcm_rs_version();
        return value == nullptr ? std::string() : std::string(value);
    }

    struct Point2
    {
        float x = 0.0f;
        float y = 0.0f;

        Point2() = default;
        Point2(float x_value, float y_value) : x(x_value), y(y_value) {}

        static Point2 zero()
        {
            return Point2();
        }

        Point2 scaled_by(float factor) const
        {
            return Point2(x * factor, y * factor);
        }

        Point2 translated_by(const Point2 &offset) const
        {
            return Point2(x + offset.x, y + offset.y);
        }

        Point2 relative_to(const Point2 &origin) const
        {
            return Point2(x - origin.x, y - origin.y);
        }

        float distance_to(const Point2 &other) const
        {
            const float dx = x - other.x;
            const float dy = y - other.y;
            return std::sqrt(dx * dx + dy * dy);
        }
    };

    inline std::vector<VvcmRsPoint2> to_raw_points(const std::vector<Point2> &points)
    {
        std::vector<VvcmRsPoint2> raw;
        raw.reserve(points.size());
        for (const auto &point : points)
        {
            raw.push_back(VvcmRsPoint2{point.x, point.y});
        }
        return raw;
    }

    struct Point3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        Point3() = default;
        Point3(float x_value, float y_value, float z_value) : x(x_value), y(y_value), z(z_value) {}

        static Point3 zero()
        {
            return Point3();
        }

        Point3 translated_xy_by(const Point2 &offset) const
        {
            return Point3(x + offset.x, y + offset.y, z);
        }

        Point3 relative_xy_to(const Point2 &origin) const
        {
            return Point3(x - origin.x, y - origin.y, z);
        }

        float distance_to(const Point3 &other) const
        {
            const float dx = x - other.x;
            const float dy = y - other.y;
            const float dz = z - other.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    };

    struct FkSolution
    {
        bool stable = false;
        Point3 po{};
        Point2 vo{};
        std::vector<size_t> taut_cables{};
    };

    class FkSolutions
    {
    public:
        FkSolutions() = default;

        explicit FkSolutions(std::vector<FkSolution> solutions) : solutions_(std::move(solutions)) {}

        bool empty() const
        {
            return solutions_.empty();
        }

        size_t all_count() const
        {
            return solutions_.size();
        }

        size_t stable_count() const
        {
            size_t count = 0;
            for (const auto &solution : solutions_)
            {
                if (solution.stable)
                {
                    ++count;
                }
            }
            return count;
        }

        const std::vector<FkSolution> &solutions() const
        {
            return solutions_;
        }

        std::vector<FkSolution> stable() const
        {
            std::vector<FkSolution> result;
            for (const auto &solution : solutions_)
            {
                if (solution.stable)
                {
                    result.push_back(solution);
                }
            }
            return result;
        }

        std::pair<size_t, FkSolution> closest_stable_to(const Point3 &reference) const
        {
            bool found = false;
            size_t best_index = 0;
            FkSolution best_solution{};
            float best_distance = 0.0f;

            for (size_t index = 0; index < solutions_.size(); ++index)
            {
                const auto &solution = solutions_[index];
                if (!solution.stable)
                {
                    continue;
                }

                const float distance = solution.po.distance_to(reference);
                if (!found || distance < best_distance)
                {
                    found = true;
                    best_index = index;
                    best_solution = solution;
                    best_distance = distance;
                }
            }

            if (!found)
            {
                throw Error("no stable VVCM solution found");
            }

            return std::make_pair(best_index, best_solution);
        }

    private:
        std::vector<FkSolution> solutions_{};
    };

    class VvcmFk
    {
    public:
        VvcmFk(size_t robot_count, float hold_height, const std::vector<Point2> &sheet)
        {
            std::vector<VvcmRsPoint2> raw_sheet = vvcm_rs::to_raw_points(sheet);
            throw_on_error(vvcm_rs_fk_new(
                robot_count,
                hold_height,
                raw_sheet.empty() ? nullptr : raw_sheet.data(),
                raw_sheet.size(),
                &handle_));
        }

        ~VvcmFk()
        {
            if (handle_ != nullptr)
            {
                vvcm_rs_fk_free(handle_);
            }
        }

        VvcmFk(const VvcmFk &) = delete;
        VvcmFk &operator=(const VvcmFk &) = delete;

        VvcmFk(VvcmFk &&other) noexcept : handle_(other.handle_)
        {
            other.handle_ = nullptr;
        }

        VvcmFk &operator=(VvcmFk &&other) noexcept
        {
            if (this != &other)
            {
                if (handle_ != nullptr)
                {
                    vvcm_rs_fk_free(handle_);
                }
                handle_ = other.handle_;
                other.handle_ = nullptr;
            }
            return *this;
        }

        FkSolutions update_stable_solutions(const std::vector<Point2> &formation)
        {
            std::vector<VvcmRsPoint2> raw_formation = vvcm_rs::to_raw_points(formation);
            throw_on_error(vvcm_rs_fk_update_stable_solutions(
                handle_,
                raw_formation.empty() ? nullptr : raw_formation.data(),
                raw_formation.size()));
            return solutions();
        }

        size_t robot_count() const
        {
            size_t count = 0;
            throw_on_error(vvcm_rs_fk_robot_count(handle_, &count));
            return count;
        }

        float hold_height() const
        {
            float hold_height_value = 0.0f;
            throw_on_error(vvcm_rs_fk_hold_height(handle_, &hold_height_value));
            return hold_height_value;
        }

        size_t solution_count() const
        {
            size_t count = 0;
            throw_on_error(vvcm_rs_fk_solution_count(handle_, &count));
            return count;
        }

        size_t stable_solution_count() const
        {
            size_t count = 0;
            throw_on_error(vvcm_rs_fk_stable_solution_count(handle_, &count));
            return count;
        }

        FkSolutions solutions() const
        {
            const size_t count = solution_count();

            std::vector<FkSolution> solutions;
            solutions.reserve(count);
            for (size_t index = 0; index < count; ++index)
            {
                VvcmRsFkSolution raw{};
                throw_on_error(vvcm_rs_fk_solution_at(handle_, index, &raw));
                solutions.push_back(from_raw(raw));
            }

            return FkSolutions(std::move(solutions));
        }

    private:
        static FkSolution from_raw(const VvcmRsFkSolution &raw)
        {
            FkSolution solution;
            solution.stable = raw.stable != 0;
            solution.po = Point3(raw.po.x, raw.po.y, raw.po.z);
            solution.vo = Point2(raw.vo.x, raw.vo.y);

            if (raw.taut_cable_count != 0)
            {
                if (raw.taut_cables == nullptr)
                {
                    throw Error("malformed taut cable list returned by vvcm-rs");
                }

                solution.taut_cables.assign(raw.taut_cables, raw.taut_cables + raw.taut_cable_count);
            }

            return solution;
        }

        VvcmRsFk *handle_ = nullptr;
    };

    class VvcmSimulation
    {
    public:
        VvcmSimulation(
            size_t robot_count,
            float hold_height,
            const std::vector<Point2> &sheet,
            const std::vector<Point2> &initial_formation,
            Point3 po_initial = Point3::zero(),
            float dt = 1.0f / 30.0f)
        {
            std::vector<VvcmRsPoint2> raw_sheet = vvcm_rs::to_raw_points(sheet);
            std::vector<VvcmRsPoint2> raw_formation = vvcm_rs::to_raw_points(initial_formation);
            throw_on_error(vvcm_rs_simulation_new(
                robot_count,
                hold_height,
                raw_sheet.empty() ? nullptr : raw_sheet.data(),
                raw_sheet.size(),
                raw_formation.empty() ? nullptr : raw_formation.data(),
                raw_formation.size(),
                VvcmRsPoint3{po_initial.x, po_initial.y, po_initial.z},
                dt,
                &handle_));
        }

        ~VvcmSimulation()
        {
            if (handle_ != nullptr)
            {
                vvcm_rs_simulation_free(handle_);
            }
        }

        VvcmSimulation(const VvcmSimulation &) = delete;
        VvcmSimulation &operator=(const VvcmSimulation &) = delete;

        VvcmSimulation(VvcmSimulation &&other) noexcept : handle_(other.handle_)
        {
            other.handle_ = nullptr;
        }

        VvcmSimulation &operator=(VvcmSimulation &&other) noexcept
        {
            if (this != &other)
            {
                if (handle_ != nullptr)
                {
                    vvcm_rs_simulation_free(handle_);
                }
                handle_ = other.handle_;
                other.handle_ = nullptr;
            }
            return *this;
        }

        void set_velocity(const std::vector<Point2> &velocity)
        {
            std::vector<VvcmRsPoint2> raw_velocity = vvcm_rs::to_raw_points(velocity);
            throw_on_error(vvcm_rs_simulation_set_velocity(
                handle_,
                raw_velocity.empty() ? nullptr : raw_velocity.data(),
                raw_velocity.size()));
        }

        void step()
        {
            throw_on_error(vvcm_rs_simulation_step(handle_));
        }

        Point2 global_position() const
        {
            VvcmRsPoint2 point{};
            throw_on_error(vvcm_rs_simulation_global_position(handle_, &point));
            return Point2(point.x, point.y);
        }

        Point3 object_position() const
        {
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_simulation_object_position(handle_, &point));
            return Point3(point.x, point.y, point.z);
        }

        Point3 absolute_object_position() const
        {
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_simulation_absolute_object_position(handle_, &point));
            return Point3(point.x, point.y, point.z);
        }

        bool has_solution_index() const
        {
            uint8_t has_value = 0;
            size_t index = 0;
            throw_on_error(vvcm_rs_simulation_solution_index(handle_, &has_value, &index));
            return has_value != 0;
        }

        size_t solution_index() const
        {
            uint8_t has_value = 0;
            size_t index = 0;
            throw_on_error(vvcm_rs_simulation_solution_index(handle_, &has_value, &index));
            if (has_value == 0)
            {
                throw Error("simulation has no selected solution");
            }
            return index;
        }

        std::vector<Point2> formation() const
        {
            return read_points(
                [&]() {
                    size_t count = 0;
                    throw_on_error(vvcm_rs_simulation_formation_count(handle_, &count));
                    return count;
                },
                [&](size_t index, VvcmRsPoint2 *point) {
                    return vvcm_rs_simulation_formation_point_at(handle_, index, point);
                });
        }

        std::vector<Point2> absolute_formation() const
        {
            return read_points(
                [&]() {
                    size_t count = 0;
                    throw_on_error(vvcm_rs_simulation_formation_count(handle_, &count));
                    return count;
                },
                [&](size_t index, VvcmRsPoint2 *point) {
                    return vvcm_rs_simulation_absolute_formation_point_at(handle_, index, point);
                });
        }

        std::vector<Point2> velocity() const
        {
            return read_points(
                [&]() {
                    size_t count = 0;
                    throw_on_error(vvcm_rs_simulation_formation_count(handle_, &count));
                    return count;
                },
                [&](size_t index, VvcmRsPoint2 *point) {
                    return vvcm_rs_simulation_velocity_point_at(handle_, index, point);
                });
        }

        std::vector<size_t> taut_cables() const
        {
            size_t count = 0;
            throw_on_error(vvcm_rs_simulation_taut_cable_count(handle_, &count));
            std::vector<size_t> cables(count);
            for (size_t index = 0; index < count; ++index)
            {
                throw_on_error(vvcm_rs_simulation_taut_cable_at(handle_, index, &cables[index]));
            }
            return cables;
        }

        float dt() const
        {
            float value = 0.0f;
            throw_on_error(vvcm_rs_simulation_dt(handle_, &value));
            return value;
        }

    private:
        template <typename CountFn, typename PointFn>
        static std::vector<Point2> read_points(CountFn &&count_fn, PointFn &&point_fn)
        {
            const size_t count = count_fn();
            std::vector<Point2> points;
            points.reserve(count);

            for (size_t index = 0; index < count; ++index)
            {
                VvcmRsPoint2 point{};
                throw_on_error(point_fn(index, &point));
                points.push_back(Point2(point.x, point.y));
            }

            return points;
        }

        VvcmRsSimulation *handle_ = nullptr;
    };

    class VvcmManualSimulation
    {
    public:
        VvcmManualSimulation(size_t robot_count, float hold_height, const std::vector<Point2> &sheet)
        {
            std::vector<VvcmRsPoint2> raw_sheet = vvcm_rs::to_raw_points(sheet);
            throw_on_error(vvcm_rs_manual_simulation_new(
                robot_count,
                hold_height,
                raw_sheet.empty() ? nullptr : raw_sheet.data(),
                raw_sheet.size(),
                &handle_));
        }

        ~VvcmManualSimulation()
        {
            if (handle_ != nullptr)
            {
                vvcm_rs_manual_simulation_free(handle_);
            }
        }

        VvcmManualSimulation(const VvcmManualSimulation &) = delete;
        VvcmManualSimulation &operator=(const VvcmManualSimulation &) = delete;

        VvcmManualSimulation(VvcmManualSimulation &&other) noexcept : handle_(other.handle_)
        {
            other.handle_ = nullptr;
        }

        VvcmManualSimulation &operator=(VvcmManualSimulation &&other) noexcept
        {
            if (this != &other)
            {
                if (handle_ != nullptr)
                {
                    vvcm_rs_manual_simulation_free(handle_);
                }
                handle_ = other.handle_;
                other.handle_ = nullptr;
            }
            return *this;
        }

        Point3 init(const std::vector<Point2> &formation, Point3 po_initial = Point3::zero())
        {
            std::vector<VvcmRsPoint2> raw_formation = vvcm_rs::to_raw_points(formation);
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_manual_simulation_init(
                handle_,
                raw_formation.empty() ? nullptr : raw_formation.data(),
                raw_formation.size(),
                VvcmRsPoint3{po_initial.x, po_initial.y, po_initial.z},
                &point));
            return Point3(point.x, point.y, point.z);
        }

        Point3 get_new_stable_solution(const std::vector<Point2> &formation)
        {
            std::vector<VvcmRsPoint2> raw_formation = vvcm_rs::to_raw_points(formation);
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_manual_simulation_get_new_stable_solution(
                handle_,
                raw_formation.empty() ? nullptr : raw_formation.data(),
                raw_formation.size(),
                &point));
            return Point3(point.x, point.y, point.z);
        }

        Point2 global_position() const
        {
            VvcmRsPoint2 point{};
            throw_on_error(vvcm_rs_manual_simulation_global_position(handle_, &point));
            return Point2(point.x, point.y);
        }

        bool has_formation() const
        {
            uint8_t has_value = 0;
            throw_on_error(vvcm_rs_manual_simulation_has_formation(handle_, &has_value));
            return has_value != 0;
        }

        std::vector<Point2> formation() const
        {
            return read_points(
                [&]() {
                    size_t count = 0;
                    throw_on_error(vvcm_rs_manual_simulation_formation_count(handle_, &count));
                    return count;
                },
                [&](size_t index, VvcmRsPoint2 *point) {
                    return vvcm_rs_manual_simulation_formation_point_at(handle_, index, point);
                });
        }

        bool has_object_position() const
        {
            uint8_t has_value = 0;
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_manual_simulation_object_position(handle_, &has_value, &point));
            return has_value != 0;
        }

        Point3 object_position() const
        {
            uint8_t has_value = 0;
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_manual_simulation_object_position(handle_, &has_value, &point));
            if (has_value == 0)
            {
                throw Error("manual simulation is not initialized");
            }
            return Point3(point.x, point.y, point.z);
        }

        bool has_absolute_object_position() const
        {
            uint8_t has_value = 0;
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_manual_simulation_absolute_object_position(
                handle_,
                &has_value,
                &point));
            return has_value != 0;
        }

        Point3 absolute_object_position() const
        {
            uint8_t has_value = 0;
            VvcmRsPoint3 point{};
            throw_on_error(vvcm_rs_manual_simulation_absolute_object_position(
                handle_,
                &has_value,
                &point));
            if (has_value == 0)
            {
                throw Error("manual simulation is not initialized");
            }
            return Point3(point.x, point.y, point.z);
        }

        bool has_solution_index() const
        {
            uint8_t has_value = 0;
            size_t index = 0;
            throw_on_error(vvcm_rs_manual_simulation_solution_index(handle_, &has_value, &index));
            return has_value != 0;
        }

        size_t solution_index() const
        {
            uint8_t has_value = 0;
            size_t index = 0;
            throw_on_error(vvcm_rs_manual_simulation_solution_index(handle_, &has_value, &index));
            if (has_value == 0)
            {
                throw Error("manual simulation has no selected solution");
            }
            return index;
        }

        std::vector<size_t> taut_cables() const
        {
            size_t count = 0;
            throw_on_error(vvcm_rs_manual_simulation_taut_cable_count(handle_, &count));
            std::vector<size_t> cables(count);
            for (size_t index = 0; index < count; ++index)
            {
                throw_on_error(
                    vvcm_rs_manual_simulation_taut_cable_at(handle_, index, &cables[index]));
            }
            return cables;
        }

    private:
        template <typename CountFn, typename PointFn>
        static std::vector<Point2> read_points(CountFn &&count_fn, PointFn &&point_fn)
        {
            const size_t count = count_fn();
            std::vector<Point2> points;
            points.reserve(count);

            for (size_t index = 0; index < count; ++index)
            {
                VvcmRsPoint2 point{};
                throw_on_error(point_fn(index, &point));
                points.push_back(Point2(point.x, point.y));
            }

            return points;
        }

        VvcmRsManualSimulation *handle_ = nullptr;
    };
} // namespace vvcm_rs

#endif
