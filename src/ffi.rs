#![allow(missing_docs)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]

//! C ABI exported for C and C++ consumers.
//!
//! This module is intentionally the only place in the crate that works with raw
//! pointers. The public Rust API remains built from safe domain types, while the
//! C ABI converts foreign buffers into those domain values before calling the
//! solver.

use crate::{
    Point2 as CorePoint2, Point3 as CorePoint3, RobotFormation, SheetShape, VvcmError, VvcmFk,
    VvcmManualSimulation, VvcmSimulation,
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
#[derive(Debug, Clone, Copy)]
pub struct VvcmRsFkSolution {
    pub stable: u8,
    pub po: VvcmRsPoint3,
    pub vo: VvcmRsPoint2,
    pub taut_cables: *const usize,
    pub taut_cable_count: usize,
}

impl Default for VvcmRsFkSolution {
    fn default() -> Self {
        Self {
            stable: 0,
            po: VvcmRsPoint3::default(),
            vo: VvcmRsPoint2::default(),
            taut_cables: ptr::null(),
            taut_cable_count: 0,
        }
    }
}

pub struct VvcmRsFk {
    inner: VvcmFk,
}

pub struct VvcmRsSimulation {
    inner: VvcmSimulation,
}

pub struct VvcmRsManualSimulation {
    inner: VvcmManualSimulation,
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
    robot_count: usize,
    hold_height: f32,
    sheet_vertices: *const VvcmRsPoint2,
    sheet_vertex_count: usize,
    out_fk: *mut *mut VvcmRsFk,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let out_fk = out_mut(out_fk, "out_fk")?;
        *out_fk = ptr::null_mut();

        let sheet = sheet_from_raw(sheet_vertices, sheet_vertex_count)?;
        let fk = VvcmFk::new(robot_count, hold_height, sheet)?;
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
    formation_points: *const VvcmRsPoint2,
    formation_point_count: usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let fk = fk_mut(fk)?;
        let formation = formation_from_raw(formation_points, formation_point_count)?;
        fk.inner.update_stable_solutions(formation)?;
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
        let solution = fk.inner.solutions().solutions.get(index).ok_or_else(|| {
            FfiError::InvalidArgument(format!("solution index {index} is out of range"))
        })?;

        *out_mut(out_solution, "out_solution")? = solution_to_ffi(solution);
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_new(
    robot_count: usize,
    hold_height: f32,
    sheet_vertices: *const VvcmRsPoint2,
    sheet_vertex_count: usize,
    initial_formation_points: *const VvcmRsPoint2,
    initial_formation_point_count: usize,
    po_initial: VvcmRsPoint3,
    dt: f32,
    out_simulation: *mut *mut VvcmRsSimulation,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let out_simulation = out_mut(out_simulation, "out_simulation")?;
        *out_simulation = ptr::null_mut();

        let sheet = sheet_from_raw(sheet_vertices, sheet_vertex_count)?;
        let initial_formation =
            formation_from_raw(initial_formation_points, initial_formation_point_count)?;
        let simulation = VvcmSimulation::new(
            robot_count,
            hold_height,
            sheet,
            initial_formation,
            core_point3(po_initial),
            dt,
        )?;

        *out_simulation = Box::into_raw(Box::new(VvcmRsSimulation { inner: simulation }));
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
    velocity_points: *const VvcmRsPoint2,
    velocity_point_count: usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_mut(simulation)?;
        let velocity = formation_from_raw(velocity_points, velocity_point_count)?;
        simulation.inner.set_velocity(velocity)?;
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
    out_point: *mut VvcmRsPoint2,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_point, "out_point")? = ffi_point2(simulation.inner.global_position());
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_object_position(
    simulation: *const VvcmRsSimulation,
    out_point: *mut VvcmRsPoint3,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_point, "out_point")? = ffi_point3(simulation.inner.object_position());
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_absolute_object_position(
    simulation: *const VvcmRsSimulation,
    out_point: *mut VvcmRsPoint3,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_point, "out_point")? = ffi_point3(simulation.inner.absolute_object_position());
        Ok(())
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
pub extern "C" fn vvcm_rs_simulation_formation_count(
    simulation: *const VvcmRsSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? = simulation.inner.formation().len();
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_formation_point_at(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_point: *mut VvcmRsPoint2,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_formation_point(simulation.inner.formation(), index, out_point)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_absolute_formation_point_at(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_point: *mut VvcmRsPoint2,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        let formation = simulation.inner.absolute_formation();
        write_formation_point(&formation, index, out_point)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_simulation_velocity_point_at(
    simulation: *const VvcmRsSimulation,
    index: usize,
    out_point: *mut VvcmRsPoint2,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = simulation_ref(simulation)?;
        write_formation_point(simulation.inner.velocity(), index, out_point)
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
pub extern "C" fn vvcm_rs_manual_simulation_new(
    robot_count: usize,
    hold_height: f32,
    sheet_vertices: *const VvcmRsPoint2,
    sheet_vertex_count: usize,
    out_simulation: *mut *mut VvcmRsManualSimulation,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let out_simulation = out_mut(out_simulation, "out_simulation")?;
        *out_simulation = ptr::null_mut();

        let sheet = sheet_from_raw(sheet_vertices, sheet_vertex_count)?;
        let simulation = VvcmManualSimulation::new(robot_count, hold_height, sheet)?;

        *out_simulation = Box::into_raw(Box::new(VvcmRsManualSimulation { inner: simulation }));
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
    formation_points: *const VvcmRsPoint2,
    formation_point_count: usize,
    po_initial: VvcmRsPoint3,
    out_point: *mut VvcmRsPoint3,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_mut(simulation)?;
        let formation = formation_from_raw(formation_points, formation_point_count)?;
        let point = simulation.inner.init(formation, core_point3(po_initial))?;
        *out_mut(out_point, "out_point")? = ffi_point3(point);
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_get_new_stable_solution(
    simulation: *mut VvcmRsManualSimulation,
    formation_points: *const VvcmRsPoint2,
    formation_point_count: usize,
    out_point: *mut VvcmRsPoint3,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_mut(simulation)?;
        let formation = formation_from_raw(formation_points, formation_point_count)?;
        let point = simulation.inner.get_new_stable_solution(formation)?;
        *out_mut(out_point, "out_point")? = ffi_point3(point);
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_global_position(
    simulation: *const VvcmRsManualSimulation,
    out_point: *mut VvcmRsPoint2,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        *out_mut(out_point, "out_point")? = ffi_point2(simulation.inner.global_position());
        Ok(())
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
pub extern "C" fn vvcm_rs_manual_simulation_formation_count(
    simulation: *const VvcmRsManualSimulation,
    out_count: *mut usize,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        *out_mut(out_count, "out_count")? =
            simulation.inner.formation().map_or(0, RobotFormation::len);
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_formation_point_at(
    simulation: *const VvcmRsManualSimulation,
    index: usize,
    out_point: *mut VvcmRsPoint2,
) -> VvcmRsErrorCode {
    run_ffi(|| {
        let simulation = manual_simulation_ref(simulation)?;
        let formation = simulation.inner.formation().ok_or_else(|| {
            FfiError::InvalidArgument("manual simulation is not initialized".to_string())
        })?;
        write_formation_point(formation, index, out_point)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn vvcm_rs_manual_simulation_object_position(
    simulation: *const VvcmRsManualSimulation,
    out_has_value: *mut u8,
    out_point: *mut VvcmRsPoint3,
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
    out_point: *mut VvcmRsPoint3,
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

fn raw_points<'a>(
    points: *const VvcmRsPoint2,
    point_count: usize,
    context: &'static str,
) -> Result<&'a [VvcmRsPoint2], FfiError> {
    if points.is_null() {
        if point_count == 0 {
            Ok(&[])
        } else {
            Err(FfiError::NullPointer(context))
        }
    } else {
        Ok(unsafe { slice::from_raw_parts(points, point_count) })
    }
}

fn formation_from_raw(
    points: *const VvcmRsPoint2,
    point_count: usize,
) -> Result<RobotFormation, FfiError> {
    RobotFormation::new(
        raw_points(points, point_count, "formation points")?
            .iter()
            .copied()
            .map(core_point2)
            .collect(),
    )
    .map_err(FfiError::from)
}

fn sheet_from_raw(
    vertices: *const VvcmRsPoint2,
    vertex_count: usize,
) -> Result<SheetShape, FfiError> {
    SheetShape::new(
        raw_points(vertices, vertex_count, "sheet vertices")?
            .iter()
            .copied()
            .map(core_point2)
            .collect(),
    )
    .map_err(FfiError::from)
}

fn core_point2(point: VvcmRsPoint2) -> CorePoint2 {
    CorePoint2::new(point.x, point.y)
}

fn core_point3(point: VvcmRsPoint3) -> CorePoint3 {
    CorePoint3::new(point.x, point.y, point.z)
}

fn ffi_point2(point: CorePoint2) -> VvcmRsPoint2 {
    VvcmRsPoint2 {
        x: point.x,
        y: point.y,
    }
}

fn ffi_point3(point: CorePoint3) -> VvcmRsPoint3 {
    VvcmRsPoint3 {
        x: point.x,
        y: point.y,
        z: point.z,
    }
}

fn solution_to_ffi(solution: &crate::FkSolution) -> VvcmRsFkSolution {
    VvcmRsFkSolution {
        stable: u8::from(solution.stable),
        po: ffi_point3(solution.po),
        vo: ffi_point2(solution.vo),
        taut_cables: if solution.taut_cables.is_empty() {
            ptr::null()
        } else {
            solution.taut_cables.as_ptr()
        },
        taut_cable_count: solution.taut_cables.len(),
    }
}

fn write_formation_point(
    formation: &RobotFormation,
    index: usize,
    out_point: *mut VvcmRsPoint2,
) -> Result<(), FfiError> {
    let point = formation.points().get(index).ok_or_else(|| {
        FfiError::InvalidArgument(format!("formation point index {index} is out of range"))
    })?;
    *out_mut(out_point, "out_point")? = ffi_point2(*point);
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
    point: Option<CorePoint3>,
    out_has_value: *mut u8,
    out_point: *mut VvcmRsPoint3,
) -> Result<(), FfiError> {
    *out_mut(out_has_value, "out_has_value")? = u8::from(point.is_some());
    *out_mut(out_point, "out_point")? = point.map_or_else(VvcmRsPoint3::default, ffi_point3);
    Ok(())
}
