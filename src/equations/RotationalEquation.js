import {Vec3} from "../math/Vec3";
import {Equation} from "./Equation";

const tmpVec1 = new Vec3();
const tmpVec2 = new Vec3();

/**
 * Rotational constraint. Works to keep the local vectors orthogonal to each other in world space.
 * @class RotationalEquation
 * @author schteppe
 */
export class RotationalEquation extends Equation {
    axisA;
    axisB;
    maxAngle;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Object} [options]
     * @param {Vec3} [options.axisA]
     * @param {Vec3} [options.axisB]
     * @param {number} [options.maxForce]
     */
    constructor(bodyA, bodyB, options = {}) {
        const {
            maxForce = 1e6,
            axisA,
            axisB
        } = options;

        super(bodyA, bodyB, -maxForce, maxForce);

        this.axisA = new Vec3(1, 0, 0);
        this.axisB = new Vec3(0, 1, 0);
        this.maxAngle = Math.PI / 2;

        if (axisA) {
            this.axisA.copy(axisA);
        }

        if (axisB) {
            this.axisB.copy(axisB);
        }
    }

    computeB(h) {
        const a = this.a,
            b = this.b,

            ni = this.axisA,
            nj = this.axisB,

            nixnj = tmpVec1,
            njxni = tmpVec2,

            GA = this.jacobianElementA,
            GB = this.jacobianElementB;

        // Caluclate cross products
        ni.cross(nj, nixnj);
        nj.cross(ni, njxni);

        // g = ni * nj
        // gdot = (nj x ni) * wi + (ni x nj) * wj
        // G = [0 njxni 0 nixnj]
        // W = [vi wi vj wj]
        GA.rotational.copy(njxni);
        GB.rotational.copy(nixnj);

        const g = Math.cos(this.maxAngle) - ni.dot(nj),
            GW = this.computeGW(),
            GiMf = this.computeGiMf();

        return -g * a - GW * b - h * GiMf;
    }
}

