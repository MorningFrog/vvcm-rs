# @morningfrog/vvcm-rs

WebAssembly bindings for `vvcm-rs`, a VVCM forward-kinematics and simulation library for multi-robot transporting systems with a deformable sheet.

## Installation

```shell
npm install @morningfrog/vvcm-rs
```

The unscoped mirror is also published as `vvcm-rs`:

```shell
npm install vvcm-rs
```

## Usage

```ts
import { VvcmFk } from "@morningfrog/vvcm-rs";

// Robot formation in row-major [x0, y0, x1, y1, ...] order, in millimeters.
const formation = new Float32Array([
  213.7, 122.7,
  804.6, 37.2,
  904.0, 550.0,
  439.3, 715.9,
]);

// Unfolded sheet vertices in the sheet's local frame, using the same row-major layout.
const sheet = new Float32Array([
  -316.1, -421.9,
  803.4, -384.1,
  746.1, 712.8,
  -367.3, 664.2,
]);

// Create the FK solver with a 1000 mm hold height.
// The robot count is inferred from sheet.length / 2.
const fk = new VvcmFk(1000, sheet);

// Solve all candidate equilibria. Each solution owns its own tautCables and
// lambdaValues arrays.
const solutions = fk.updateStableSolutions(formation);

console.log(`all solutions: ${solutions.allCount}`);
console.log(`stable solutions: ${solutions.stableCount}`);

// Filter stable branches just like calling stable() in the Rust/Python/C++ APIs.
for (const solution of solutions.solutions.filter((item) => item.stable)) {
  // lambdaValues[i] belongs to tautCables[i] on the same solution.
  const lambdaValues = solution.lambdaValues.map((value) => value.toFixed(3)).join(", ");
  console.log(
    solution.po,
    solution.vo,
    solution.tautCables,
    `lambda=[${lambdaValues}]`,
  );
}

// Free the underlying WASM allocation when the solver is no longer needed.
fk.free();
```

Formation, sheet, and velocity inputs are row-major `Float32Array` values laid out as `[x0, y0, x1, y1, ...]`. Point-3 inputs such as initial object references are `Float32Array` values with length 3. FK outputs are solution objects; each solution carries its own `tautCables` and matching taut-only `lambdaValues`.

## TypeScript

The package ships a hand-written `index.d.ts` file with the stable public API, including `VvcmFk`, `VvcmSimulation`, `VvcmManualSimulation`, `FkSolutionsOutput`, `FkSolutionOutput`, typed-array input aliases, and `VvcmError`.
