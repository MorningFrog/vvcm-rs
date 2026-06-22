export type PointMatrixInput = Float32Array;
export type Point3Input = Float32Array;

export type VvcmErrorCode =
  | "INVALID_ARGUMENT"
  | "DIMENSION_MISMATCH"
  | "INFEASIBLE_FORMATION"
  | "NO_SOLUTION"
  | "NO_STABLE_SOLUTION";

export interface VvcmBaseError extends Error {
  name: "VvcmError";
  code: VvcmErrorCode;
}

export interface VvcmInvalidArgumentError extends VvcmBaseError {
  code: "INVALID_ARGUMENT";
}

export interface VvcmDimensionMismatchError extends VvcmBaseError {
  code: "DIMENSION_MISMATCH";
  context: string;
  expected: number;
  actual: number;
}

export interface VvcmInfeasibleFormationError extends VvcmBaseError {
  code: "INFEASIBLE_FORMATION";
}

export interface VvcmNoSolutionError extends VvcmBaseError {
  code: "NO_SOLUTION";
}

export interface VvcmNoStableSolutionError extends VvcmBaseError {
  code: "NO_STABLE_SOLUTION";
}

export type VvcmError =
  | VvcmInvalidArgumentError
  | VvcmDimensionMismatchError
  | VvcmInfeasibleFormationError
  | VvcmNoSolutionError
  | VvcmNoStableSolutionError;

export interface Point2Output {
  x: number;
  y: number;
}

export interface Point3Output {
  x: number;
  y: number;
  z: number;
}

export interface FkSolutionOutput {
  stable: boolean;
  po: Point3Output;
  vo: Point2Output;
  tautCables: number[];
  lambdaValues: number[];
}

export interface FkSolutionsOutput {
  solutions: FkSolutionOutput[];
  allCount: number;
  stableCount: number;
}

export function version(): string;

export class VvcmFk {
  constructor(holdHeight: number, sheet: PointMatrixInput);
  updateStableSolutions(formation: PointMatrixInput): FkSolutionsOutput;
  solutions(): FkSolutionsOutput;
  robotCount(): number;
  holdHeight(): number;
  free(): void;
}

export class VvcmSimulation {
  constructor(
    holdHeight: number,
    sheet: PointMatrixInput,
    initialFormation: PointMatrixInput,
    poInitial: Point3Input,
    dt: number,
  );
  setVelocity(velocity: PointMatrixInput): void;
  step(): void;
  absoluteFormation(): Float32Array;
  absoluteObjectPosition(): Float32Array;
  globalPosition(): Float32Array;
  formation(): Float32Array;
  objectPosition(): Float32Array;
  tautCables(): Uint32Array;
  solutionIndex(): number | null;
  dt(): number;
  velocity(): Float32Array;
  solutions(): FkSolutionsOutput;
  free(): void;
}

export class VvcmManualSimulation {
  constructor(holdHeight: number, sheet: PointMatrixInput);
  init(formation: PointMatrixInput, poInitial: Point3Input): Float32Array;
  getNewStableSolution(formation: PointMatrixInput): Float32Array;
  globalPosition(): Float32Array;
  hasFormation(): boolean;
  formation(): Float32Array | null;
  objectPosition(): Float32Array | null;
  absoluteObjectPosition(): Float32Array | null;
  tautCables(): Uint32Array;
  solutionIndex(): number | null;
  solutions(): FkSolutionsOutput;
  free(): void;
}
