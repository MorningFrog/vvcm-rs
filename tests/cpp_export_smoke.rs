use std::env;
use std::ffi::OsString;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

#[test]
fn cpp_export_smoke() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let source = manifest_dir.join("tests").join("cpp_export_smoke.cpp");
    let include_dir = manifest_dir.join("include");
    let target_dir = current_target_dir();

    build_native_library(&manifest_dir, &target_dir);

    let host = rust_host_triple();
    let cc_out_dir = target_dir.join("cpp_export_smoke_cc_out");
    fs::create_dir_all(&cc_out_dir).expect("failed to create C++ compiler scratch dir");

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .host(&host)
        .target(&host)
        .opt_level(0)
        .out_dir(&cc_out_dir)
        .cargo_metadata(false)
        .cargo_warnings(false)
        .cargo_output(false);
    let compiler = build.get_compiler();
    let exe_path = output_executable_path(&target_dir);
    let _ = fs::remove_file(&exe_path);
    let _ = fs::remove_file(output_object_path(&target_dir));
    let _ = fs::remove_file(target_dir.join("vvcm_rs_cpp_smoke.ilk"));
    let _ = fs::remove_file(target_dir.join("vvcm_rs_cpp_smoke.pdb"));

    let mut command = compiler.to_command();
    command.current_dir(&target_dir);
    if compiler.is_like_msvc() {
        command.arg("/nologo");
        command.arg("/std:c++17");
        command.arg("/EHsc");
        command.arg(format!("/I{}", include_dir.display()));
        command.arg(format!("/Fe:{}", exe_path.display()));
        command.arg(source.as_os_str());
        command.arg(import_library_path(&target_dir).as_os_str());
        command.arg(format!("/Fo:{}", output_object_path(&target_dir).display()));
    } else {
        command.arg("-std=c++17");
        command.arg("-I");
        command.arg(&include_dir);
        command.arg(&source);
        command.arg("-L");
        command.arg(&target_dir);
        command.arg("-lvvcm_rs");
        command.arg("-o");
        command.arg(&exe_path);
    }

    let output = command.output().expect("failed to invoke C++ compiler");
    if !output.status.success() {
        panic!(
            "C++ compilation failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let mut run = Command::new(&exe_path);
    if cfg!(windows) {
        // The executable and the Rust cdylib live in the same directory, so no
        // PATH adjustment is needed on Windows.
    } else if cfg!(target_os = "macos") {
        prepend_library_path(&mut run, "DYLD_LIBRARY_PATH", &target_dir);
    } else {
        prepend_library_path(&mut run, "LD_LIBRARY_PATH", &target_dir);
    }

    let output = run.output().expect("failed to run C++ smoke test");
    if !output.status.success() {
        panic!(
            "C++ smoke test failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

fn build_native_library(manifest_dir: &Path, target_dir: &Path) {
    let cargo = env::var_os("CARGO").unwrap_or_else(|| OsString::from("cargo"));
    let mut command = Command::new(cargo);
    command.arg("build");
    command.arg("--lib");
    command.arg("--manifest-path");
    command.arg(manifest_dir.join("Cargo.toml"));
    if is_release_profile(target_dir) {
        command.arg("--release");
    }

    let output = command
        .output()
        .expect("failed to invoke cargo build --lib");
    if !output.status.success() {
        panic!(
            "cargo build --lib failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let native_library = dynamic_library_path(target_dir);
    if !native_library.exists() {
        panic!("expected native library at {}", native_library.display());
    }

    if cfg!(windows) {
        let import_library = import_library_path(target_dir);
        if !import_library.exists() {
            panic!("expected import library at {}", import_library.display());
        }
    }
}

fn current_target_dir() -> PathBuf {
    let mut path = env::current_exe().expect("current exe path");
    path.pop();
    path.pop();
    path
}

fn is_release_profile(target_dir: &Path) -> bool {
    target_dir
        .file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| name == "release")
}

fn dynamic_library_path(target_dir: &Path) -> PathBuf {
    if cfg!(windows) {
        target_dir.join("vvcm_rs.dll")
    } else if cfg!(target_os = "macos") {
        target_dir.join("libvvcm_rs.dylib")
    } else {
        target_dir.join("libvvcm_rs.so")
    }
}

fn import_library_path(target_dir: &Path) -> PathBuf {
    if cfg!(windows) {
        target_dir.join("vvcm_rs.dll.lib")
    } else {
        dynamic_library_path(target_dir)
    }
}

fn output_executable_path(target_dir: &Path) -> PathBuf {
    if cfg!(windows) {
        target_dir.join("vvcm_rs_cpp_smoke.exe")
    } else {
        target_dir.join("vvcm_rs_cpp_smoke")
    }
}

fn output_object_path(target_dir: &Path) -> PathBuf {
    if cfg!(windows) {
        target_dir.join("vvcm_rs_cpp_smoke.obj")
    } else {
        target_dir.join("vvcm_rs_cpp_smoke.o")
    }
}

fn prepend_library_path(command: &mut Command, var: &str, target_dir: &Path) {
    let current = env::var_os(var).unwrap_or_default();
    let mut value = OsString::from(target_dir.as_os_str());
    if !current.is_empty() {
        value.push(if cfg!(windows) { ";" } else { ":" });
        value.push(current);
    }
    command.env(var, value);
}

fn rust_host_triple() -> String {
    let output = Command::new("rustc")
        .arg("-vV")
        .output()
        .expect("failed to query rustc host triple");

    if !output.status.success() {
        panic!(
            "rustc -vV failed:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    for line in stdout.lines() {
        if let Some(host) = line.strip_prefix("host: ") {
            return host.trim().to_string();
        }
    }

    panic!("unable to determine rustc host triple from:\n{stdout}");
}
