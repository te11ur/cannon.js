import {Equation} from "./Equation";
import {Vec3} from "../math/Vec3";

const FrictionEquation_computeB_temp1 = new Vec3();
const FrictionEquation_computeB_temp2 = new Vec3();

/**
 * Constrains the slipping in a contact along a tangent
 * @class FrictionEquation
 * @author schteppe
 */
export class FrictionEquation extends Equation {

    ri = new Vec3();
    rj = new Vec3();
    t = new Vec3(); // tangent
    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Number} slipForce should be +-F_friction = +-mu * F_normal = +-mu * m * g
     */
    constructor(bodyA, bodyB, slipForce) {
        super(bodyA, bodyB, -slipForce, slipForce);
    }

    computeB(h) {
        const b = this.b,
            ri = this.ri,
            rj = this.rj,
            rixt = FrictionEquation_computeB_temp1,
            rjxt = FrictionEquation_computeB_temp2,
            t = this.t;

        // Caluclate cross products
        ri.cross(t, rixt);
        rj.cross(t, rjxt);

        // G = [-t -rixt t rjxt]
        // And remember, this is a pure velocity constraint, g is always zero!
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB;
        t.negate(GA.spatial);
        rixt.negate(GA.rotational);
        GB.spatial.copy(t);
        GB.rotational.copy(rjxt);

        const GW = this.computeGW();
        const GiMf = this.computeGiMf();

        return -GW * b - h * GiMf;
    }
}
