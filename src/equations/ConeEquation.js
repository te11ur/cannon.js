import {Vec3} from "../math/Vec3";
import {Equation} from "./Equation";

const tmpVec1 = new Vec3();
const tmpVec2 = new Vec3();

/**
 * Cone equation. Works to keep the given body world vectors aligned, or tilted within a given angle from each other.
 * @class ConeEquation
 * @author schteppe
 */
export class ConeEquation extends Equation {
    /**
     * @property {Vec3} axisA
     */
    axisA;
    /**
     * @property {Vec3} axisB
     */
    axisB;
    /**
     * The cone angle to keep
     * @property {number} angle
     */
    angle = 0;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Object} [options]
     * @param {Vec3} [options.axisA] Local axis in A
     * @param {Vec3} [options.axisB] Local axis in B
     * @param {Vec3} [options.angle] The "cone angle" to keep
     * @param {number} [options.maxForce=1e6]
     */
    constructor(bodyA, bodyB, options) {
        const {
            maxForce = 1e6,
            angle = 0,
            axisA,
            axisB
        } = Object.assign({}, options);

        super(bodyA, bodyB, -maxForce, maxForce);

        this.axisA = new Vec3(1, 0, 0);
        this.axisB = new Vec3(0, 1, 0);
        this.angle = angle;

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

        // The angle between two vector is:
        // cos(theta) = a * b / (length(a) * length(b) = { len(a) = len(b) = 1 } = a * b

        // g = a * b
        // gdot = (b x a) * wi + (a x b) * wj
        // G = [0 bxa 0 axb]
        // W = [vi wi vj wj]
        GA.rotational.copy(njxni);
        GB.rotational.copy(nixnj);

        const g = Math.cos(this.angle) - ni.dot(nj),
            GW = this.computeGW(),
            GiMf = this.computeGiMf();

        const B = -g * a - GW * b - h * GiMf;

        return B;
    }
}

