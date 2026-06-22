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

typedef struct VvcmRsMat2f
{
    const float *data;
    size_t rows;
    size_t stride;
} VvcmRsMat2f;

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
    size_t taut_cable_count;
    size_t lambda_value_count;
} VvcmRsFkSolution;

typedef struct VvcmRsFk VvcmRsFk;
typedef struct VvcmRsSimulation VvcmRsSimulation;
typedef struct VvcmRsManualSimulation VvcmRsManualSimulation;

const char *vvcm_rs_version(void);
const char *vvcm_rs_last_error_message(void);
const char *vvcm_rs_error_message(VvcmRsErrorCode code);

VvcmRsErrorCode vvcm_rs_fk_new(
    float hold_height,
    VvcmRsMat2f sheet,
    VvcmRsFk **out_fk);
void vvcm_rs_fk_free(VvcmRsFk *fk);
VvcmRsErrorCode vvcm_rs_fk_update_stable_solutions(
    VvcmRsFk *fk,
    VvcmRsMat2f formation);
VvcmRsErrorCode vvcm_rs_fk_robot_count(VvcmRsFk *fk, size_t *out_count);
VvcmRsErrorCode vvcm_rs_fk_hold_height(VvcmRsFk *fk, float *out_hold_height);
VvcmRsErrorCode vvcm_rs_fk_solution_count(VvcmRsFk *fk, size_t *out_count);
VvcmRsErrorCode vvcm_rs_fk_stable_solution_count(VvcmRsFk *fk, size_t *out_count);
VvcmRsErrorCode vvcm_rs_fk_solution_at(
    VvcmRsFk *fk,
    size_t index,
    VvcmRsFkSolution *out_solution);
VvcmRsErrorCode vvcm_rs_fk_solution_taut_cables(
    VvcmRsFk *fk,
    size_t index,
    size_t *out_cables,
    size_t *inout_count);
VvcmRsErrorCode vvcm_rs_fk_solution_lambda_values(
    VvcmRsFk *fk,
    size_t index,
    float *out_values,
    size_t *inout_count);

VvcmRsErrorCode vvcm_rs_simulation_new(
    float hold_height,
    VvcmRsMat2f sheet,
    VvcmRsMat2f initial_formation,
    const float *po_initial,
    float dt,
    VvcmRsSimulation **out_simulation);
void vvcm_rs_simulation_free(VvcmRsSimulation *simulation);
VvcmRsErrorCode vvcm_rs_simulation_set_velocity(
    VvcmRsSimulation *simulation,
    VvcmRsMat2f velocity);
VvcmRsErrorCode vvcm_rs_simulation_step(VvcmRsSimulation *simulation);
VvcmRsErrorCode vvcm_rs_simulation_global_position(
    VvcmRsSimulation *simulation,
    float *out_point2);
VvcmRsErrorCode vvcm_rs_simulation_object_position(
    VvcmRsSimulation *simulation,
    float *out_point3);
VvcmRsErrorCode vvcm_rs_simulation_absolute_object_position(
    VvcmRsSimulation *simulation,
    float *out_point3);
VvcmRsErrorCode vvcm_rs_simulation_solution_index(
    VvcmRsSimulation *simulation,
    uint8_t *out_has_value,
    size_t *out_index);
VvcmRsErrorCode vvcm_rs_simulation_formation_view(
    VvcmRsSimulation *simulation,
    VvcmRsMat2f *out_view);
VvcmRsErrorCode vvcm_rs_simulation_absolute_formation_view(
    VvcmRsSimulation *simulation,
    VvcmRsMat2f *out_view);
VvcmRsErrorCode vvcm_rs_simulation_velocity_view(
    VvcmRsSimulation *simulation,
    VvcmRsMat2f *out_view);
VvcmRsErrorCode vvcm_rs_simulation_taut_cable_count(
    VvcmRsSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_simulation_taut_cable_at(
    VvcmRsSimulation *simulation,
    size_t index,
    size_t *out_cable);
VvcmRsErrorCode vvcm_rs_simulation_dt(VvcmRsSimulation *simulation, float *out_dt);
VvcmRsErrorCode vvcm_rs_simulation_solution_count(
    VvcmRsSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_simulation_stable_solution_count(
    VvcmRsSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_simulation_solution_at(
    VvcmRsSimulation *simulation,
    size_t index,
    VvcmRsFkSolution *out_solution);
VvcmRsErrorCode vvcm_rs_simulation_solution_taut_cables(
    VvcmRsSimulation *simulation,
    size_t index,
    size_t *out_cables,
    size_t *inout_count);
VvcmRsErrorCode vvcm_rs_simulation_solution_lambda_values(
    VvcmRsSimulation *simulation,
    size_t index,
    float *out_values,
    size_t *inout_count);

VvcmRsErrorCode vvcm_rs_manual_simulation_new(
    float hold_height,
    VvcmRsMat2f sheet,
    VvcmRsManualSimulation **out_simulation);
void vvcm_rs_manual_simulation_free(VvcmRsManualSimulation *simulation);
VvcmRsErrorCode vvcm_rs_manual_simulation_init(
    VvcmRsManualSimulation *simulation,
    VvcmRsMat2f formation,
    const float *po_initial,
    float *out_point3);
VvcmRsErrorCode vvcm_rs_manual_simulation_get_new_stable_solution(
    VvcmRsManualSimulation *simulation,
    VvcmRsMat2f formation,
    float *out_point3);
VvcmRsErrorCode vvcm_rs_manual_simulation_global_position(
    VvcmRsManualSimulation *simulation,
    float *out_point2);
VvcmRsErrorCode vvcm_rs_manual_simulation_has_formation(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value);
VvcmRsErrorCode vvcm_rs_manual_simulation_formation_view(
    VvcmRsManualSimulation *simulation,
    VvcmRsMat2f *out_view);
VvcmRsErrorCode vvcm_rs_manual_simulation_object_position(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value,
    float *out_point3);
VvcmRsErrorCode vvcm_rs_manual_simulation_absolute_object_position(
    VvcmRsManualSimulation *simulation,
    uint8_t *out_has_value,
    float *out_point3);
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
VvcmRsErrorCode vvcm_rs_manual_simulation_solution_count(
    VvcmRsManualSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_manual_simulation_stable_solution_count(
    VvcmRsManualSimulation *simulation,
    size_t *out_count);
VvcmRsErrorCode vvcm_rs_manual_simulation_solution_at(
    VvcmRsManualSimulation *simulation,
    size_t index,
    VvcmRsFkSolution *out_solution);
VvcmRsErrorCode vvcm_rs_manual_simulation_solution_taut_cables(
    VvcmRsManualSimulation *simulation,
    size_t index,
    size_t *out_cables,
    size_t *inout_count);
VvcmRsErrorCode vvcm_rs_manual_simulation_solution_lambda_values(
    VvcmRsManualSimulation *simulation,
    size_t index,
    float *out_values,
    size_t *inout_count);

#ifdef __cplusplus
}
#endif

#endif
