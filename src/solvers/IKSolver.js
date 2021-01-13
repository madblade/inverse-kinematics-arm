
import { CCD }        from './CCD';
import { FABRIK }     from './FABRIK';

const Solver = {
    CCD: 'CCD',
    FABRIK: 'FABRIK'
};

let IKSolver = function()
{
    this.ccdSolver = new CCD();
    this.fabrikSolver = new FABRIK();
};

IKSolver.prototype.customSolve = function(
    chain, target, constraints
)
{
    this.solve(Solver.FABRIK, chain, target, 3, null);
    this.solve(Solver.CCD, chain, target, 3, constraints);
};

IKSolver.prototype.solve = function(
    solver, chain, target, iterations, constraints, activateConstraints
)
{
    switch (solver)
    {
        case Solver.CCD:
            this.ccdSolver.solve(chain, target, iterations, constraints, activateConstraints);
            break;
        case Solver.FABRIK:
            this.fabrikSolver.solve(chain, target, iterations, constraints, activateConstraints);
    }
};

export { IKSolver, Solver };
