import {Constraint} from "./Constraint";
import {ContactEquation} from "../equations/ContactEquation";
import {Vec3} from "../math/Vec3";

/**
 * Connects two bodies at given offset points.
 * @class PointToPointConstraint
 *
 * @example
 *     var bodyA = new Body({ mass: 1 });
 *     var bodyB = new Body({ mass: 1 });
 *     bodyA.position.set(-1, 0, 0);
 *     bodyB.position.set(1, 0, 0);
 *     bodyA.addShape(shapeA);
 *     bodyB.addShape(shapeB);
 *     world.addBody(bodyA);
 *     world.addBody(bodyB);
 *     var localPivotA = new Vec3(1, 0, 0);
 *     var localPivotB = new Vec3(-1, 0, 0);
 *     var constraint = new PointToPointConstraint(bodyA, localPivotA, bodyB, localPivotB);
 *     world.addConstraint(constraint);
 */
export class PointToPointConstraint extends Constraint {

    /**
     * Pivot, defined locally in bodyA.
     * @property {Vec3} pivotA
     */
    pivotA;

    /**
     * Pivot, defined locally in bodyB.
     * @property {Vec3} pivotB
     */
    pivotB;

    /**
     * @property {ContactEquation} equationX
     */
    equationX;

    /**
     * @property {ContactEquation} equationY
     */
    equationY;

    /**
     * @property {ContactEquation} equationZ
     */
    equationZ;

    /**
     * @param {Body} bodyA
     * @param {Vec3} pivotA The point relative to the center of mass of bodyA which bodyA is constrained to.
     * @param {Body} bodyB Body that will be constrained in a similar way to the same point as bodyA. We will therefore get a link between bodyA and bodyB. If not specified, bodyA will be constrained to a static point.
     * @param {Vec3} pivotB See pivotA.
     * @param {Number} maxForce The maximum force that should be applied to constrain the bodies.
     */
    constructor(bodyA, pivotA, bodyB, pivotB, maxForce = 1e6) {
        super(bodyA, bodyB);

        this.pivotA = pivotA ? pivotA.clone() : new Vec3();
        this.pivotB = pivotB ? pivotB.clone() : new Vec3();

        const x = this.equationX = new ContactEquation(bodyA, bodyB);
        const y = this.equationY = new ContactEquation(bodyA, bodyB);
        const z = this.equationZ = new ContactEquation(bodyA, bodyB);

        // Equations to be fed to the solver
        this.equations.push(x, y, z);

        // Make the equations bidirectional
        x.minForce = y.minForce = z.minForce = -maxForce;
        x.maxForce = y.maxForce = z.maxForce = maxForce;

        x.ni.set(1, 0, 0);
        y.ni.set(0, 1, 0);
        z.ni.set(0, 0, 1);

    }

    update() {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const x = this.equationX;
        const y = this.equationY;
        const z = this.equationZ;

        // Rotate the pivots to world space
        bodyA.quaternion.vmult(this.pivotA, x.ri);
        bodyB.quaternion.vmult(this.pivotB, x.rj);

        y.ri.copy(x.ri);
        y.rj.copy(x.rj);
        z.ri.copy(x.ri);
        z.rj.copy(x.rj);
    }
}