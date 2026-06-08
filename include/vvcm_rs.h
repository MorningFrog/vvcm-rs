#ifndef VVCM_RS_H
#define VVCM_RS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t VvcmRsErrorCode;

enum
{
    VVCM_RS_ERROR_OK = 0,
    VVCM_RS_ERROR_NULL_POINTER = 1,
    VVCM_RS_ERROR_INVALID_ARGUMENT = 2,
    VVCM_RS_ERROR_DIMENSION_MISMATCH = 3,
    VVCM_RS_ERROR_NO_SOLUTION = 4,
    VVCM_RS_ERROR_NO_STABLE_SOLUTION = 5,
    VVCM_RS_ERROR_INFEASIBLE_FORMATION = 6,
    VVCM_RS_ERROR_PANIC = 7
};

typedef struct VvcmRsPoint2
{
    float x;
    float y;
} VvcmRsPoint2;

typedef struct VvcmRsPoint3
{
    float x;
    float y;
    float z;
} VvcmRsPoint3;

typedef struct VvcmRsFkSolution
{
    uint8_t stable;
    VvcmRsPoint3 po;
    VvcmRsPoint2 vo;
    const size_t *taut_cables;
    size_t taut_cable_count;
} VvcmRsFkSolution;

typedef struct VvcmRsFk VvcmRsFk;
typedef struct VvcmRsSimulation VvcmRsSimulation;
typedef struct VvcmRsManualSimulation VvcmRsManualSimulation;

const char *vvcm_rs_version(void);
const char *vvcm_rs_last_error_message(void);
const char *vvcm_rs_error_message(VvcmRsErrorCode code);

VvcmRsErrorCode vvcm_rs_fk_new(
    size_t robot_count,
    float hold_height,
    const VvcmRsPoint2 *sheet_vertices,
    size_t sheet_vertex_count,
    VvcmRsFk **out_fk);
void vvcm_rs_fk_free(VvcmRsFk *fk);
VvcmRsErrorCode vvcm_rs_fk_update_stable_solutions(
    VvcmRsFk *fk,
    const VvcmRsPoint2 *formation_points,
    size_t formation_point_count);
VvcmRsErrorCode vvcm_rs_fk_robot_count(VvcmRsFk *fk, size_t *out_count);
VvcmRsErrorCode vvcm_rs_fk_hold_height(VvcmRsFk *fk, float *out_hold_height);
VvcmRsErrorCode vvcm_rs_fk_solution_count(VvcmRsFk *fk, size_t *out_count);
VvcmRsErrorCode vvcm_rs_fk_stable_solution_count(VvcmRsFk *fk, size_t *out_count);
VvcmRsErrorCode vvcm_rs_fk_solution_at(
    VvcmRsFk *fk,
    size_t index,
    VvcmRsFkSolution *out_solution);

VvcmRsErrorCode vvcm_rs_simulation_new(
    size_t robot_count,
    float hold_height,
    const VvcmRsPoint2 *sheet_vertices,
    size_t sheet_vertex_count,
    const VvcmRsPoint2 *initial_formation_points,
    size_t initial_formation_point_count,
    VvcmRsPoint3 po_initial,
    float dt,
    VvcmRsSimulation **out_simulation);
void vvcm_rs_simulation_free(VvcmRsSimulation *simulation);
VvcmRsErrorCode vvcm_rs_simulation_set_velocity(
    VvcmRsSimulation *simulation,
    const VvcmRsPoint2 *velocity_points,
    size_t velocity_point_count);
VvcmRsErrorCode vvcm_rs_simulation_step(VvcmRsSimulation *simulation);
VvcmRsErrorCode vvcm_rs_simulation_global_position(
    VvcmRsSimulation *simulation,
    VvcmRsPoint2 *out_point);
VvcmRsErrorCode vvcm_rs_simulation_object_position(
    VvcmRsSimulation *simulation,
    VvcmRsPoint3 *out_point);
VvcmRsErrorCode vvcm_rs_simulation_absolute_object_position(
    VvcmRsSimulation *simulation,
    VvcmRsPoint3 *out_point);
VvcmRsErrorCode vvcm_rs_simulation_solution_index(
    VvcmRsSimulation *simulation,
    uint8_t *out_has_value,
    size_t *out_index);
VvcmRsErrorCode vvcm_rs_simulation_formation_count(
    VvcmRsSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_simulation_formation_point_at(
    VvcmRsSimulation *simulation,
    size_t index,
    VvcmRsPoint2 *out_point);
VvcmRsErrorCode vvcm_rs_simulation_absolute_formation_point_at(
    VvcmRsSimulation *simulation,
    size_t index,
    VvcmRsPoint2 *out_point);
VvcmRsErrorCode vvcm_rs_simulation_velocity_point_at(
    VvcmRsSimulation *simulation,
    size_t index,
    VvcmRsPoint2 *out_point);
VvcmRsErrorCode vvcm_rs_simulation_taut_cable_count(
    VvcmRsSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_simulation_taut_cable_at(
    VvcmRsSimulation *simulation,
    size_t index,
    size_t *out_cable);
VvcmRsErrorCode vvcm_rs_simulation_dt(VvcmRsSimulation *simulation, float *out_dt);

VvcmRsErrorCode vvcm_rs_manual_simulation_new(
    size_t robot_count,
    float hold_height,
    const VvcmRsPoint2 *sheet_vertices,
    size_t sheet_vertex_count,
    VvcmRsManualSimulation **out_simulation);
void vvcm_rs_manual_simulation_free(VvcmRsManualSimulation *simulation);
VvcmRsErrorCode vvcm_rs_manual_simulation_init(
    VvcmRsManualSimulation *simulation,
    const VvcmRsPoint2 *formation_points,
    size_t formation_point_count,
    VvcmRsPoint3 po_initial,
    VvcmRsPoint3 *out_point);
VvcmRsErrorCode vvcm_rs_manual_simulation_get_new_stable_solution(
    VvcmRsManualSimulation *simulation,
    const VvcmRsPoint2 *formation_points,
    size_t formation_point_count,
    VvcmRsPoint3 *out_point);
VvcmRsErrorCode vvcm_rs_manual_simulation_global_position(
    VvcmRsManualSimulation *simulation,
    VvcmRsPoint2 *out_point);
VvcmRsErrorCode vvcm_rs_manual_simulation_has_formation(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value);
VvcmRsErrorCode vvcm_rs_manual_simulation_formation_count(
    VvcmRsManualSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_manual_simulation_formation_point_at(
    VvcmRsManualSimulation *simulation,
    size_t index,
    VvcmRsPoint2 *out_point);
VvcmRsErrorCode vvcm_rs_manual_simulation_object_position(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value,
    VvcmRsPoint3 *out_point);
VvcmRsErrorCode vvcm_rs_manual_simulation_absolute_object_position(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value,
    VvcmRsPoint3 *out_point);
VvcmRsErrorCode vvcm_rs_manual_simulation_solution_index(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value,
    size_t *out_index);
VvcmRsErrorCode vvcm_rs_manual_simulation_taut_cable_count(
    VvcmRsManualSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_manual_simulation_taut_cable_at(
    VvcmRsManualSimulation *simulation,
    size_t index,
    size_t *out_cable);

#ifdef __cplusplus
}
#endif

#endif
