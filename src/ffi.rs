#![allow(missing_docs)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]

//! C ABI exported for C and C++ consumers.
//!
//! The ABI uses row-major matrix views at the language boundary. Rust callers
//! use `nalgebra` points directly; C, C++, and other native callers pass
//! contiguous or strided `n x 2` buffers.

use crate::{
    FkSolution, FkSolutions, Point2, Point3, Vector2, VvcmError, VvcmFk, VvcmManualSimulation,
    VvcmSimulation,
};
use std::cell::RefCell;
use std::ffi::CString;
use std::os::raw::c_char;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::ptr;
use std::slice;

pub type VvcmRsErrorCode = i32;

const ERROR_OK: VvcmRsErrorCode = 0;
const ERROR_NULL_POINTER: VvcmRsErrorCode = 1;
const ERROR_INVALID_ARGUMENT: VvcmRsErrorCode = 2;
const ERROR_DIMENSION_MISMATCH: VvcmRsErrorCode = 3;
const ERROR_NO_SOLUTION: VvcmRsErrorCode = 4;
const ERROR_NO_STABLE_SOLUTION: VvcmRsErrorCode = 5;
const ERROR_INFEASIBLE_FORMATION: VvcmRsErrorCode = 6;
const ERROR_PANIC: VvcmRsErrorCode = 7;

thread_local! {
    static LAST_ERROR: RefCell<CString> =
        RefCell::new(CString::new("").expect("empty string has no interior NUL"));
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VvcmRsMat2f {
    pub data: *const f32,
    pub rows: usize,
    pub stride: usize,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VvcmRsPoint2 {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VvcmRsPoint3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VvcmRsFkSolution {
    pub stable: u8,
    pub po: VvcmRsPoint3,
    pub vo: VvcmRsPoint2,
    pub taut_cable_count: usize,
    pub lambda_value_count: usize,
}

pub struct VvcmRsFk {
    inner: VvcmFk,
}

pub struct VvcmRsSimulation {
    inner: VvcmSimulation,
    formation_buffer: Matrix2Buffer,
    absolute_formation_buffer: Matrix2Buffer,
    velocity_buffer: Matrix2Buffer,
}

pub struct VvcmRsManualSimulation {
    inner: VvcmManualSimulation,
    formation_buffer: Matrix2Buffer,
}

#[derive(Debug)]
enum FfiError {
    NullPointer(&'static str),
    InvalidArgument(String),
    Core(VvcmError),
    Panic,
}

impl FfiError {
    fn code(&self) -> VvcmRsErrorCode {
        match self {
            Self::NullPointer(_) => ERROR_NULL_POINTER,
            Self::InvalidArgument(_) => ERROR_INVALID_ARGUMENT,
            Self::Core(VvcmError::DimensionMismatch { .. }) => ERROR_DIMENSION_MISMATCH,
            Self::Core(VvcmError::NoSolution) => ERROR_NO_SOLUTION,
            Self::Core(VvcmError::NoStableSolution) => ERROR_NO_STABLE_SOLUTION,
            Self::Core(VvcmError::InfeasibleFormation) => ERROR_INFEASIBLE_FORMATION,
            Self::Panic => ERROR_PANIC,
        }
    }

    fn message(&self) -> String {
        match self {
            Self::NullPointer(context) => format!("null pointer passed for {context}"),
            Self::InvalidArgument(message) => message.clone(),
            Self::Core(error) => error.to_string(),
            Self::Panic => "panic while running vvcm-rs FFI call".to_string(),
        }
    }
}

impl From<VvcmError> for FfiError {
    fn from(value: VvcmError) -> Self {
        Self::Core(value)
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0")
        .as_ptr()
        .cast::<c_char>()
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_last_error_message() -> *const c_char {
    LAST_ERROR.with(|message| message.borrow().as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_error_message(code: VvcmRsErrorCode) -> *const c_char {
    match code {
        ERROR_OK => c_string_literal("ok"),
        ERROR_NULL_POINTER => c_string_literal("null pointer"),
        ERROR_INVALID_ARGUMENT => c_string_literal("invalid argument"),
        ERROR_DIMENSION_MISMATCH => c_string_literal("dimension mismatch"),
        ERROR_NO_SOLUTION => c_string_literal("no VVCM solution found"),
        ERROR_NO_STABLE_SOLUTION => c_string_literal("no stable VVCM solution found"),
        ERROR_INFEASIBLE_FORMATION => c_string_literal("infeasible robot formation"),
        ERROR_PANIC => c_string_literal("panic while running vvcm-rs FFI call"),
        _ => c_string_literal("unknown vvcm-rs error"),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_new(
    hold_height: f32,
    sheet: VvcmRsMat2f,
    out_fk: *mut *mut VvcmRsFk,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let out_fk = out_mut(out_fk, "out_fk")?;
        *out_fk = ptr::null_mut();

        let fk = VvcmFk::new(hold_height, points_from_mat2(sheet, "sheet")?)?;
        *out_fk = Box::into_raw(Box::new(VvcmRsFk { inner: fk }));

        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_free(fk: *mut VvcmRsFk) {
    if !fk.is_null() {
        unsafe {
            drop(Box::from_raw(fk));
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_update_stable_solutions(
    fk: *mut VvcmRsFk,
    formation: VvcmRsMat2f,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_mut(fk)?;
        let formation = points_from_mat2(formation, "formation")?;
        fk.inner.update_stable_solutions(&formation)?;
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_robot_count(
    fk: *const VvcmRsFk,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        *out_mut(out_count, "out_count")? = fk.inner.robot_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_hold_height(
    fk: *const VvcmRsFk,
    out_hold_height: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        *out_mut(out_hold_height, "out_hold_height")? = fk.inner.hold_height();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_solution_count(
    fk: *const VvcmRsFk,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        *out_mut(out_count, "out_count")? = fk.inner.solutions().all_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_stable_solution_count(
    fk: *const VvcmRsFk,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        *out_mut(out_count, "out_count")? = fk.inner.solutions().stable_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_solution_at(
    fk: *const VvcmRsFk,
    index: usize,
    out_solution: *mut VvcmRsFkSolution,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        let solution = solution_at(fk.inner.solutions(), index)?;
        write_solution(solution, out_solution)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_solution_taut_cables(
    fk: *const VvcmRsFk,
    index: usize,
    out_cables: *mut usize,
    inout_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        let solution = solution_at(fk.inner.solutions(), index)?;
        copy_usize_values(&solution.taut_cables, out_cables, inout_count, "out_cables")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_fk_solution_lambda_values(
    fk: *const VvcmRsFk,
    index: usize,
    out_values: *mut f32,
    inout_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_ref(fk)?;
        let solution = solution_at(fk.inner.solutions(), index)?;
        copy_f32_values(
            &solution.lambda_values,
            out_values,
            inout_count,
            "out_values",
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_new(
    hold_height: f32,
    sheet: VvcmRsMat2f,
    initial_formation: VvcmRsMat2f,
    po_initial: *const f32,
    dt: f32,
    out_simulation: *mut *mut VvcmRsSimulation,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let out_simulation = out_mut(out_simulation, "out_simulation")?;
        *out_simulation = ptr::null_mut();

        let sheet = points_from_mat2(sheet, "sheet")?;
        let initial_formation = points_from_mat2(initial_formation, "initial formation")?;
        let simulation = VvcmSimulation::new(
            hold_height,
            sheet,
            &initial_formation,
            point3_from_ptr(po_initial, "po_initial")?,
            dt,
        )?;

        *out_simulation = Box::into_raw(Box::new(VvcmRsSimulation {
            inner: simulation,
            formation_buffer: Matrix2Buffer::default(),
            absolute_formation_buffer: Matrix2Buffer::default(),
            velocity_buffer: Matrix2Buffer::default(),
        }));
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_free(simulation: *mut VvcmRsSimulation) {
    if !simulation.is_null() {
        unsafe {
            drop(Box::from_raw(simulation));
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_set_velocity(
    simulation: *mut VvcmRsSimulation,
    velocity: VvcmRsMat2f,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_mut(simulation)?;
        let velocity = vectors_from_mat2(velocity, "velocity")?;
        simulation.inner.set_velocity(&velocity)?;
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_step(simulation: *mut VvcmRsSimulation) -> VvcmRsErrorCode {
    run_ffi(|| {
        simulation_mut(simulation)?.inner.step()?;
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_global_position(
    simulation: *const VvcmRsSimulation,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_point2(simulation.inner.global_position(), out_point, "out_point")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_object_position(
    simulation: *const VvcmRsSimulation,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_point3(simulation.inner.object_position(), out_point, "out_point")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_absolute_object_position(
    simulation: *const VvcmRsSimulation,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_point3(
            simulation.inner.absolute_object_position(),
            out_point,
            "out_point",
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_solution_index(
    simulation: *const VvcmRsSimulation,
    out_has_value: *mut u8,
    out_index: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_optional_index(simulation.inner.solution_index(), out_has_value, out_index)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_formation_view(
    simulation: *mut VvcmRsSimulation,
    out_view: *mut VvcmRsMat2f,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_mut(simulation)?;
        simulation
            .formation_buffer
            .sync_points(simulation.inner.formation());
        *out_mut(out_view, "out_view")? = simulation.formation_buffer.view();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_absolute_formation_view(
    simulation: *mut VvcmRsSimulation,
    out_view: *mut VvcmRsMat2f,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_mut(simulation)?;
        simulation
            .absolute_formation_buffer
            .sync_points(simulation.inner.absolute_formation());
        *out_mut(out_view, "out_view")? = simulation.absolute_formation_buffer.view();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_velocity_view(
    simulation: *mut VvcmRsSimulation,
    out_view: *mut VvcmRsMat2f,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_mut(simulation)?;
        simulation
            .velocity_buffer
            .sync_vectors(simulation.inner.velocity());
        *out_mut(out_view, "out_view")? = simulation.velocity_buffer.view();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_taut_cable_count(
    simulation: *const VvcmRsSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.taut_cables().len();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_taut_cable_at(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_cable: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_taut_cable(simulation.inner.taut_cables(), index, out_cable)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_dt(
    simulation: *const VvcmRsSimulation,
    out_dt: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_dt, "out_dt")? = simulation.inner.dt();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_solution_count(
    simulation: *const VvcmRsSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.fk_engine().solutions().all_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_stable_solution_count(
    simulation: *const VvcmRsSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.fk_engine().solutions().stable_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_solution_at(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_solution: *mut VvcmRsFkSolution,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        let solution = solution_at(simulation.inner.fk_engine().solutions(), index)?;
        write_solution(solution, out_solution)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_solution_taut_cables(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_cables: *mut usize,
    inout_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        let solution = solution_at(simulation.inner.fk_engine().solutions(), index)?;
        copy_usize_values(&solution.taut_cables, out_cables, inout_count, "out_cables")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_solution_lambda_values(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_values: *mut f32,
    inout_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        let solution = solution_at(simulation.inner.fk_engine().solutions(), index)?;
        copy_f32_values(
            &solution.lambda_values,
            out_values,
            inout_count,
            "out_values",
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_new(
    hold_height: f32,
    sheet: VvcmRsMat2f,
    out_simulation: *mut *mut VvcmRsManualSimulation,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let out_simulation = out_mut(out_simulation, "out_simulation")?;
        *out_simulation = ptr::null_mut();

        let simulation = VvcmManualSimulation::new(hold_height, points_from_mat2(sheet, "sheet")?)?;

        *out_simulation = Box::into_raw(Box::new(VvcmRsManualSimulation {
            inner: simulation,
            formation_buffer: Matrix2Buffer::default(),
        }));
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_free(simulation: *mut VvcmRsManualSimulation) {
    if !simulation.is_null() {
        unsafe {
            drop(Box::from_raw(simulation));
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_init(
    simulation: *mut VvcmRsManualSimulation,
    formation: VvcmRsMat2f,
    po_initial: *const f32,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_mut(simulation)?;
        let formation = points_from_mat2(formation, "formation")?;
        let point = simulation
            .inner
            .init(&formation, point3_from_ptr(po_initial, "po_initial")?)?;
        write_point3(point, out_point, "out_point")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_get_new_stable_solution(
    simulation: *mut VvcmRsManualSimulation,
    formation: VvcmRsMat2f,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_mut(simulation)?;
        let formation = points_from_mat2(formation, "formation")?;
        let point = simulation.inner.get_new_stable_solution(&formation)?;
        write_point3(point, out_point, "out_point")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_global_position(
    simulation: *const VvcmRsManualSimulation,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        write_point2(simulation.inner.global_position(), out_point, "out_point")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_has_formation(
    simulation: *const VvcmRsManualSimulation,
    out_has_value: *mut u8,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        *out_mut(out_has_value, "out_has_value")? =
            u8::from(simulation.inner.formation().is_some());
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_formation_view(
    simulation: *mut VvcmRsManualSimulation,
    out_view: *mut VvcmRsMat2f,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_mut(simulation)?;
        let formation = simulation.inner.formation().ok_or_else(|| {
            FfiError::InvalidArgument("manual simulation is not initialized".to_string())
        })?;
        simulation.formation_buffer.sync_points(formation);
        *out_mut(out_view, "out_view")? = simulation.formation_buffer.view();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_object_position(
    simulation: *const VvcmRsManualSimulation,
    out_has_value: *mut u8,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        write_optional_point3(simulation.inner.object_position(), out_has_value, out_point)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_absolute_object_position(
    simulation: *const VvcmRsManualSimulation,
    out_has_value: *mut u8,
    out_point: *mut f32,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        write_optional_point3(
            simulation.inner.absolute_object_position(),
            out_has_value,
            out_point,
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_solution_index(
    simulation: *const VvcmRsManualSimulation,
    out_has_value: *mut u8,
    out_index: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        write_optional_index(simulation.inner.solution_index(), out_has_value, out_index)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_taut_cable_count(
    simulation: *const VvcmRsManualSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.taut_cables().len();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_taut_cable_at(
    simulation: *const VvcmRsManualSimulation,
    index: usize,
    out_cable: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        write_taut_cable(simulation.inner.taut_cables(), index, out_cable)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_solution_count(
    simulation: *const VvcmRsManualSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.fk_engine().solutions().all_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_stable_solution_count(
    simulation: *const VvcmRsManualSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.fk_engine().solutions().stable_count();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_solution_at(
    simulation: *const VvcmRsManualSimulation,
    index: usize,
    out_solution: *mut VvcmRsFkSolution,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        let solution = solution_at(simulation.inner.fk_engine().solutions(), index)?;
        write_solution(solution, out_solution)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_solution_taut_cables(
    simulation: *const VvcmRsManualSimulation,
    index: usize,
    out_cables: *mut usize,
    inout_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        let solution = solution_at(simulation.inner.fk_engine().solutions(), index)?;
        copy_usize_values(&solution.taut_cables, out_cables, inout_count, "out_cables")
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_solution_lambda_values(
    simulation: *const VvcmRsManualSimulation,
    index: usize,
    out_values: *mut f32,
    inout_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        let solution = solution_at(simulation.inner.fk_engine().solutions(), index)?;
        copy_f32_values(
            &solution.lambda_values,
            out_values,
            inout_count,
            "out_values",
        )
    })
}

fn run_ffi(operation: impl FnOnce() -> Result<(), FfiError>) -> VvcmRsErrorCode {
    let result = catch_unwind(AssertUnwindSafe(operation)).unwrap_or(Err(FfiError::Panic));

    match result {
        Ok(()) => {
            clear_last_error();
            ERROR_OK
        }
        Err(error) => {
            let code = error.code();
            set_last_error(error.message());
            code
        }
    }
}

fn clear_last_error() {
    set_last_error("");
}

fn set_last_error(message: impl Into<String>) {
    let message = message.into().replace('\0', "\\0");
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() =
            CString::new(message).expect("interior NUL characters were replaced above");
    });
}

fn c_string_literal(message: &'static str) -> *const c_char {
    match message {
        "ok" => c"ok".as_ptr(),
        "null pointer" => c"null pointer".as_ptr(),
        "invalid argument" => c"invalid argument".as_ptr(),
        "dimension mismatch" => c"dimension mismatch".as_ptr(),
        "no VVCM solution found" => c"no VVCM solution found".as_ptr(),
        "no stable VVCM solution found" => c"no stable VVCM solution found".as_ptr(),
        "infeasible robot formation" => c"infeasible robot formation".as_ptr(),
        "panic while running vvcm-rs FFI call" => c"panic while running vvcm-rs FFI call".as_ptr(),
        _ => c"unknown vvcm-rs error".as_ptr(),
    }
}

fn out_mut<'a, T>(ptr: *mut T, context: &'static str) -> Result<&'a mut T, FfiError> {
    if ptr.is_null() {
        Err(FfiError::NullPointer(context))
    } else {
        Ok(unsafe { &mut *ptr })
    }
}

fn fk_ref<'a>(fk: *const VvcmRsFk) -> Result<&'a VvcmRsFk, FfiError> {
    if fk.is_null() {
        Err(FfiError::NullPointer("fk"))
    } else {
        Ok(unsafe { &*fk })
    }
}

fn fk_mut<'a>(fk: *mut VvcmRsFk) -> Result<&'a mut VvcmRsFk, FfiError> {
    if fk.is_null() {
        Err(FfiError::NullPointer("fk"))
    } else {
        Ok(unsafe { &mut *fk })
    }
}

fn simulation_ref<'a>(
    simulation: *const VvcmRsSimulation,
) -> Result<&'a VvcmRsSimulation, FfiError> {
    if simulation.is_null() {
        Err(FfiError::NullPointer("simulation"))
    } else {
        Ok(unsafe { &*simulation })
    }
}

fn simulation_mut<'a>(
    simulation: *mut VvcmRsSimulation,
) -> Result<&'a mut VvcmRsSimulation, FfiError> {
    if simulation.is_null() {
        Err(FfiError::NullPointer("simulation"))
    } else {
        Ok(unsafe { &mut *simulation })
    }
}

fn manual_simulation_ref<'a>(
    simulation: *const VvcmRsManualSimulation,
) -> Result<&'a VvcmRsManualSimulation, FfiError> {
    if simulation.is_null() {
        Err(FfiError::NullPointer("manual simulation"))
    } else {
        Ok(unsafe { &*simulation })
    }
}

fn manual_simulation_mut<'a>(
    simulation: *mut VvcmRsManualSimulation,
) -> Result<&'a mut VvcmRsManualSimulation, FfiError> {
    if simulation.is_null() {
        Err(FfiError::NullPointer("manual simulation"))
    } else {
        Ok(unsafe { &mut *simulation })
    }
}

fn mat2_row_stride(mat: VvcmRsMat2f, context: &'static str) -> Result<usize, FfiError> {
    let stride = if mat.stride == 0 { 2 } else { mat.stride };
    if stride < 2 {
        return Err(FfiError::InvalidArgument(format!(
            "{context} stride must be at least 2"
        )));
    }
    Ok(stride)
}

fn mat2_values<'a>(mat: VvcmRsMat2f, context: &'static str) -> Result<&'a [f32], FfiError> {
    if mat.rows == 0 {
        return Ok(&[]);
    }
    if mat.data.is_null() {
        return Err(FfiError::NullPointer(context));
    }

    let stride = mat2_row_stride(mat, context)?;
    let len = (mat.rows - 1)
        .checked_mul(stride)
        .and_then(|offset| offset.checked_add(2))
        .ok_or_else(|| FfiError::InvalidArgument(format!("{context} shape overflows usize")))?;
    Ok(unsafe { slice::from_raw_parts(mat.data, len) })
}

fn points_from_mat2(mat: VvcmRsMat2f, context: &'static str) -> Result<Vec<Point2>, FfiError> {
    let values = mat2_values(mat, context)?;
    let stride = mat2_row_stride(mat, context)?;
    let mut points = Vec::with_capacity(mat.rows);
    for row in 0..mat.rows {
        let offset = row * stride;
        points.push(Point2::new(values[offset], values[offset + 1]));
    }
    Ok(points)
}

fn vectors_from_mat2(mat: VvcmRsMat2f, context: &'static str) -> Result<Vec<Vector2>, FfiError> {
    let values = mat2_values(mat, context)?;
    let stride = mat2_row_stride(mat, context)?;
    let mut vectors = Vec::with_capacity(mat.rows);
    for row in 0..mat.rows {
        let offset = row * stride;
        vectors.push(Vector2::new(values[offset], values[offset + 1]));
    }
    Ok(vectors)
}

fn point3_from_ptr(ptr: *const f32, context: &'static str) -> Result<Point3, FfiError> {
    if ptr.is_null() {
        return Err(FfiError::NullPointer(context));
    }

    let values = unsafe { slice::from_raw_parts(ptr, 3) };
    Ok(Point3::new(values[0], values[1], values[2]))
}

fn write_point2(point: Point2, out_point: *mut f32, context: &'static str) -> Result<(), FfiError> {
    if out_point.is_null() {
        return Err(FfiError::NullPointer(context));
    }

    unsafe {
        *out_point.add(0) = point.x;
        *out_point.add(1) = point.y;
    }
    Ok(())
}

fn write_point3(point: Point3, out_point: *mut f32, context: &'static str) -> Result<(), FfiError> {
    if out_point.is_null() {
        return Err(FfiError::NullPointer(context));
    }

    unsafe {
        *out_point.add(0) = point.x;
        *out_point.add(1) = point.y;
        *out_point.add(2) = point.z;
    }
    Ok(())
}

fn write_taut_cable(
    taut_cables: &[usize],
    index: usize,
    out_cable: *mut usize,
) -> Result<(), FfiError> {
    let cable = taut_cables.get(index).ok_or_else(|| {
        FfiError::InvalidArgument(format!("taut cable index {index} is out of range"))
    })?;
    *out_mut(out_cable, "out_cable")? = *cable;
    Ok(())
}

fn solution_at(solutions: &FkSolutions, index: usize) -> Result<&FkSolution, FfiError> {
    solutions
        .solutions
        .get(index)
        .ok_or_else(|| FfiError::InvalidArgument(format!("solution index {index} is out of range")))
}

fn write_solution(
    solution: &FkSolution,
    out_solution: *mut VvcmRsFkSolution,
) -> Result<(), FfiError> {
    *out_mut(out_solution, "out_solution")? = VvcmRsFkSolution {
        stable: u8::from(solution.stable),
        po: VvcmRsPoint3 {
            x: solution.po.x,
            y: solution.po.y,
            z: solution.po.z,
        },
        vo: VvcmRsPoint2 {
            x: solution.vo.x,
            y: solution.vo.y,
        },
        taut_cable_count: solution.taut_cables.len(),
        lambda_value_count: solution.lambda_values.len(),
    };
    Ok(())
}

fn copy_usize_values(
    values: &[usize],
    out_values: *mut usize,
    inout_count: *mut usize,
    context: &'static str,
) -> Result<(), FfiError> {
    copy_values(values, out_values, inout_count, context)
}

fn copy_f32_values(
    values: &[f32],
    out_values: *mut f32,
    inout_count: *mut usize,
    context: &'static str,
) -> Result<(), FfiError> {
    copy_values(values, out_values, inout_count, context)
}

fn copy_values<T: Copy>(
    values: &[T],
    out_values: *mut T,
    inout_count: *mut usize,
    context: &'static str,
) -> Result<(), FfiError> {
    let count = out_mut(inout_count, "inout_count")?;
    let capacity = *count;
    *count = values.len();

    if values.is_empty() || out_values.is_null() && capacity == 0 {
        return Ok(());
    }
    if out_values.is_null() {
        return Err(FfiError::NullPointer(context));
    }
    if capacity < values.len() {
        return Err(FfiError::Core(VvcmError::DimensionMismatch {
            context,
            expected: values.len(),
            actual: capacity,
        }));
    }

    unsafe {
        ptr::copy_nonoverlapping(values.as_ptr(), out_values, values.len());
    }
    Ok(())
}

fn write_optional_index(
    index: Option<usize>,
    out_has_value: *mut u8,
    out_index: *mut usize,
) -> Result<(), FfiError> {
    *out_mut(out_has_value, "out_has_value")? = u8::from(index.is_some());
    *out_mut(out_index, "out_index")? = index.unwrap_or_default();
    Ok(())
}

fn write_optional_point3(
    point: Option<Point3>,
    out_has_value: *mut u8,
    out_point: *mut f32,
) -> Result<(), FfiError> {
    *out_mut(out_has_value, "out_has_value")? = u8::from(point.is_some());
    write_point3(
        point.unwrap_or_else(|| Point3::new(0.0, 0.0, 0.0)),
        out_point,
        "out_point",
    )
}

#[derive(Debug, Clone, Default)]
struct Matrix2Buffer {
    data: Vec<f32>,
    rows: usize,
}

impl Matrix2Buffer {
    fn sync_points(&mut self, points: &[Point2]) {
        self.rows = points.len();
        self.data.clear();
        self.data.reserve(points.len() * 2);
        for point in points {
            self.data.push(point.x);
            self.data.push(point.y);
        }
    }

    fn sync_vectors(&mut self, vectors: &[Vector2]) {
        self.rows = vectors.len();
        self.data.clear();
        self.data.reserve(vectors.len() * 2);
        for vector in vectors {
            self.data.push(vector.x);
            self.data.push(vector.y);
        }
    }

    fn view(&self) -> VvcmRsMat2f {
        VvcmRsMat2f {
            data: if self.data.is_empty() {
                ptr::null()
            } else {
                self.data.as_ptr()
            },
            rows: self.rows,
            stride: 2,
        }
    }
}
