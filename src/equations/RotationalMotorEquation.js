import {Vec3} from "../math/Vec3";
import {Equation} from "./Equation";

/**
 * Rotational motor constraint. Tries to keep the relative angular velocity of the bodies to a given value.
 * @class RotationalMotorEquation
 * @author schteppe
 */
export class RotationalMotorEquation extends Equation {
    /**
     * World oriented rotational axis
     * @property {Vec3} axisA
     */
    axisA = new Vec3();

    /**
     * World oriented rotational axis
     * @property {Vec3} axisB
     */
    axisB = new Vec3(); // World oriented rotational axis

    /**
     * Motor velocity
     * @property {Number} targetVelocity
     */
    targetVelocity = 0;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Number} maxForce
     */
    constructor(bodyA, bodyB, maxForce = 1e6) {
        super(bodyA, bodyB, -maxForce, maxForce);
    }

    computeB(h) {
        const a = this.a,
            b = this.b,
            bi = this.bi,
            bj = this.bj,

            axisA = this.axisA,
            axisB = this.axisB,

            GA = this.jacobianElementA,
            GB = this.jacobianElementB;

        // g = 0
        // gdot = axisA * wi - axisB * wj
        // gdot = G * W = G * [vi wi vj wj]
        // =>
        // G = [0 axisA 0 -axisB]

        GA.rotational.copy(axisA);
        axisB.negate(GB.rotational);

        const GW = this.computeGW() - this.targetVelocity,
            GiMf = this.computeGiMf();

        return -GW * b - h * GiMf;
    }
}

