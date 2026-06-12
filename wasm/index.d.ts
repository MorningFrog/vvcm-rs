export type Point2Input = readonly [number, number] | { readonly x: number; readonly y: number };

export type Point3Input =
  | readonly [number, number, number]
  | { readonly x: number; readonly y: number; readonly z: number };

export interface Point2 {
  x: number;
  y: number;
}

export interface Point3 {
  x: number;
  y: number;
  z: number;
}

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

export interface FkSolutionOutput {
  stable: boolean;
  po: Point3;
  vo: Point2;
  tautCables: number[];
}

export interface FkSolutionsOutput {
  solutions: FkSolutionOutput[];
  allCount: number;
  stableCount: number;
}

export function version(): string;

export class VvcmFk {
  constructor(robotCount: number, holdHeight: number, sheet: readonly Point2Input[]);
  updateStableSolutions(formation: readonly Point2Input[]): FkSolutionsOutput;
  solutions(): FkSolutionsOutput;
  stableSolutions(): FkSolutionOutput[];
  robotCount(): number;
  holdHeight(): number;
  free(): void;
}

export class VvcmSimulation {
  constructor(
    robotCount: number,
    holdHeight: number,
    sheet: readonly Point2Input[],
    initialFormation: readonly Point2Input[],
    poInitial: Point3Input,
    dt: number,
  );
  setVelocity(velocity: readonly Point2Input[]): void;
  step(): void;
  absoluteFormation(): Point2[];
  absoluteObjectPosition(): Point3;
  globalPosition(): Point2;
  formation(): Point2[];
  objectPosition(): Point3;
  tautCables(): number[];
  solutionIndex(): number | null;
  dt(): number;
  velocity(): Point2[];
  solutions(): FkSolutionsOutput;
  free(): void;
}

export class VvcmManualSimulation {
  constructor(robotCount: number, holdHeight: number, sheet: readonly Point2Input[]);
  init(formation: readonly Point2Input[], poInitial: Point3Input): Point3;
  getNewStableSolution(formation: readonly Point2Input[]): Point3;
  globalPosition(): Point2;
  hasFormation(): boolean;
  formation(): Point2[] | null;
  objectPosition(): Point3 | null;
  absoluteObjectPosition(): Point3 | null;
  tautCables(): number[];
  solutionIndex(): number | null;
  solutions(): FkSolutionsOutput;
  free(): void;
}
