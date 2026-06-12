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

const formation = [
  [213.7, 122.7],
  [804.6, 37.2],
  [904.0, 550.0],
  [439.3, 715.9],
] as const;

const sheet = [
  [-316.1, -421.9],
  [803.4, -384.1],
  [746.1, 712.8],
  [-367.3, 664.2],
] as const;

const fk = new VvcmFk(4, 1000, sheet);
const solutions = fk.updateStableSolutions(formation);

for (const solution of solutions.solutions.filter((item) => item.stable)) {
  console.log(solution.po, solution.vo, solution.tautCables);
}
```

`Point2Input` values can be `[x, y]` tuples or `{ x, y }` objects. `Point3Input` values can be `[x, y, z]` tuples or `{ x, y, z }` objects.

## TypeScript

The package ships a hand-written `index.d.ts` file with the stable public API, including `VvcmFk`, `VvcmSimulation`, `VvcmManualSimulation`, `FkSolutionsOutput`, and `VvcmError`.
