import {
  VvcmFk,
  VvcmManualSimulation,
  VvcmSimulation,
  version,
  type FkSolutionsOutput,
  type Point2Input,
  type Point3Input,
  type VvcmError,
} from "../index.js";

const formation: Point2Input[] = [
  [213.7, 122.7],
  [804.6, 37.2],
  [904.0, 550.0],
  [439.3, 715.9],
];

const sheet: Point2Input[] = [
  { x: -316.1, y: -421.9 },
  { x: 803.4, y: -384.1 },
  { x: 746.1, y: 712.8 },
  { x: -367.3, y: 664.2 },
];

const poInitial: Point3Input = [0, 0, 0];

function isVvcmError(error: unknown): error is VvcmError {
  return error instanceof Error && "code" in error;
}

function smoke(): number {
  const fk = new VvcmFk(4, 1000, sheet);
  const solutions: FkSolutionsOutput = fk.updateStableSolutions(formation);
  const stable = fk.stableSolutions();
  const robotCount: number = fk.robotCount();
  const holdHeight: number = fk.holdHeight();
  fk.free();

  const simulation = new VvcmSimulation(4, 1000, sheet, formation, poInitial, 1 / 30);
  simulation.setVelocity([
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 0],
  ]);
  simulation.step();
  const absoluteObject = simulation.absoluteObjectPosition();
  simulation.free();

  const manual = new VvcmManualSimulation(4, 1000, sheet);
  const manualObject = manual.init(formation, poInitial);
  const optionalObject = manual.objectPosition();
  manual.free();

  try {
    new VvcmFk(4, 1000, [[0, 0]]);
  } catch (error) {
    if (isVvcmError(error) && error.code === "DIMENSION_MISMATCH") {
      const expected: number = error.expected;
      return expected;
    }
  }

  return (
    version().length +
    solutions.allCount +
    stable.length +
    robotCount +
    holdHeight +
    absoluteObject.z +
    manualObject.z +
    (optionalObject?.z ?? 0)
  );
}

void smoke();
