import {
  VvcmFk,
  VvcmManualSimulation,
  VvcmSimulation,
  version,
  type FkSolutionOutput,
  type FkSolutionsOutput,
  type PointMatrixInput,
  type Point3Input,
  type VvcmError,
} from "../index.js";

const formation: PointMatrixInput = new Float32Array([
  213.7, 122.7,
  804.6, 37.2,
  904.0, 550.0,
  439.3, 715.9,
]);

const sheet: PointMatrixInput = new Float32Array([
  -316.1, -421.9,
  803.4, -384.1,
  746.1, 712.8,
  -367.3, 664.2,
]);

const poInitial: Point3Input = new Float32Array([0, 0, 0]);

function isVvcmError(error: unknown): error is VvcmError {
  return error instanceof Error && "code" in error;
}

function smoke(): number {
  const fk = new VvcmFk(1000, sheet);
  const solutions: FkSolutionsOutput = fk.updateStableSolutions(formation);
  const stableSolutions: FkSolutionOutput[] = solutions.solutions.filter((solution) => solution.stable);
  const lambdaValues: number[] = stableSolutions[0]?.lambdaValues ?? [];
  const robotCount: number = fk.robotCount();
  const holdHeight: number = fk.holdHeight();
  fk.free();

  const simulation = new VvcmSimulation(1000, sheet, formation, poInitial, 1 / 30);
  simulation.setVelocity(new Float32Array(8));
  simulation.step();
  const absoluteObject = simulation.absoluteObjectPosition();
  simulation.free();

  const manual = new VvcmManualSimulation(1000, sheet);
  const manualObject = manual.init(formation, poInitial);
  const optionalObject = manual.objectPosition();
  manual.free();

  try {
    new VvcmFk(1000, new Float32Array([0, 0]));
  } catch (error) {
    if (isVvcmError(error) && error.code === "DIMENSION_MISMATCH") {
      const expected: number = error.expected;
      return expected;
    }
  }

  return (
    version().length +
    solutions.allCount +
    solutions.stableCount +
    stableSolutions.length +
    (stableSolutions[0]?.tautCables.length ?? 0) +
    lambdaValues.length +
    robotCount +
    holdHeight +
    absoluteObject[2] +
    manualObject[2] +
    (optionalObject?.[2] ?? 0)
  );
}

void smoke();
