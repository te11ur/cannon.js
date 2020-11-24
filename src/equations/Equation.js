import {Vec3} from "../math/Vec3";
import {JacobianElement} from "../math/JacobianElement";

const iMfi = new Vec3();
const iMfj = new Vec3();
const invIi_vmult_taui = new Vec3();
const invIj_vmult_tauj = new Vec3();
const tmp = new Vec3();

const addToWlambda_temp = new Vec3();

/**
 * Equation base class
 * @class Equation
 * @author schteppe
 */
export class Equation {
    static id = 0;

    id = Equation.id++;

    /**
     * @property {number} minForce
     */
    minForce;

    /**
     * @property {number} maxForce
     */
    maxForce;

    /**
     * @property bi
     * @type {Body}
     */
    bi;

    /**
     * @property bj
     * @type {Body}
     */
    bj;

    /**
     * SPOOK parameter
     * @property {number} a
     */
    a;

    /**
     * SPOOK parameter
     * @property {number} b
     */
    b;

    /**
     * SPOOK parameter
     * @property {number} eps
     */
    eps;

    /**
     * @property {JacobianElement} jacobianElementA
     */
    jacobianElementA;

    /**
     * @property {JacobianElement} jacobianElementB
     */
    jacobianElementB;

    /**
     * @property {boolean} enabled
     * @default true
     */
    enabled;

    /**
     * A number, proportional to the force added to the bodies.
     * @property {number} multiplier
     * @readonly
     */
    multiplier;

    /**
     * @param {Body} bi
     * @param {Body} bj
     * @param {Number} minForce Minimum (read: negative max) force to be applied by the constraint.
     * @param {Number} maxForce Maximum (read: positive max) force to be applied by the constraint.
     */
    constructor(bi, bj, minForce = -1e6, maxForce = 1e6) {
        this.bi = bi;
        this.bj = bj;
        this.minForce = minForce;
        this.maxForce = maxForce;
        this.jacobianElementA = new JacobianElement();
        this.jacobianElementB = new JacobianElement();
        this.enabled = true;
        this.multiplier = 0;
        // Set typical spook params
        this.setSpookParams(1e7, 4, 1 / 60);
    }

    /**
     * Recalculates a,b,eps.
     * @method setSpookParams
     */
    setSpookParams(stiffness, relaxation, timeStep) {
        const d = relaxation,
            k = stiffness,
            h = timeStep;
        this.a = 4.0 / (h * (1 + 4 * d));
        this.b = (4.0 * d) / (1 + 4 * d);
        this.eps = 4.0 / (h * h * k * (1 + 4 * d));
    }

    /**
     * Computes the RHS of the SPOOK equation
     * @method computeB
     * @return {Number}
     */
    computeB(a, b, h) {
        const GW = this.computeGW(),
            Gq = this.computeGq(),
            GiMf = this.computeGiMf();

        return -Gq * a - GW * b - GiMf * h;
    }

    /**
     * Computes G*q, where q are the generalized body coordinates
     * @method computeGq
     * @return {Number}
     */
    computeGq() {
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB,
            bi = this.bi,
            bj = this.bj,
            xi = bi.position,
            xj = bj.position;

        return GA.spatial.dot(xi) + GB.spatial.dot(xj);
    }

    /**
     * Computes G*W, where W are the body velocities
     * @method computeGW
     * @return {Number}
     */
    computeGW() {
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB,
            bi = this.bi,
            bj = this.bj,
            vi = bi.velocity,
            vj = bj.velocity,
            wi = bi.angularVelocity,
            wj = bj.angularVelocity;

        return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
    }

    /**
     * Computes G*Wlambda, where W are the body velocities
     * @method computeGWlambda
     * @return {Number}
     */
    computeGWlambda() {
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB,
            bi = this.bi,
            bj = this.bj,
            vi = bi.vlambda,
            vj = bj.vlambda,
            wi = bi.wlambda,
            wj = bj.wlambda;

        return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
    }

    /**
     * Computes G*inv(M)*f, where M is the mass matrix with diagonal blocks for each body, and f are the forces on the bodies.
     * @method computeGiMf
     * @return {Number}
     */
    computeGiMf() {
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB,
            bi = this.bi,
            bj = this.bj,
            fi = bi.force,
            ti = bi.torque,
            fj = bj.force,
            tj = bj.torque,
            invMassi = bi.invMassSolve,
            invMassj = bj.invMassSolve;

        fi.scale(invMassi, iMfi);
        fj.scale(invMassj, iMfj);

        bi.invInertiaWorldSolve.vmult(ti, invIi_vmult_taui);
        bj.invInertiaWorldSolve.vmult(tj, invIj_vmult_tauj);

        return GA.multiplyVectors(iMfi, invIi_vmult_taui) + GB.multiplyVectors(iMfj, invIj_vmult_tauj);
    }

    /**
     * Computes G*inv(M)*G'
     * @method computeGiMGt
     * @return {Number}
     */
    computeGiMGt() {
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB,
            bi = this.bi,
            bj = this.bj,
            invMassi = bi.invMassSolve,
            invMassj = bj.invMassSolve,
            invIi = bi.invInertiaWorldSolve,
            invIj = bj.invInertiaWorldSolve;

        let result = invMassi + invMassj;

        invIi.vmult(GA.rotational, tmp);
        result += tmp.dot(GA.rotational);

        invIj.vmult(GB.rotational, tmp);
        result += tmp.dot(GB.rotational);

        return result;
    }

    /**
     * Add constraint velocity to the bodies.
     * @method addToWlambda
     * @param {Number} deltalambda
     */
    addToWlambda(deltalambda) {
        const GA = this.jacobianElementA,
            GB = this.jacobianElementB,
            bi = this.bi,
            bj = this.bj,
            temp = addToWlambda_temp;

        // Add to linear velocity
        // v_lambda += inv(M) * delta_lamba * G
        bi.vlambda.addScaledVector(bi.invMassSolve * deltalambda, GA.spatial, bi.vlambda);
        bj.vlambda.addScaledVector(bj.invMassSolve * deltalambda, GB.spatial, bj.vlambda);

        // Add to angular velocity
        bi.invInertiaWorldSolve.vmult(GA.rotational, temp);
        bi.wlambda.addScaledVector(deltalambda, temp, bi.wlambda);

        bj.invInertiaWorldSolve.vmult(GB.rotational, temp);
        bj.wlambda.addScaledVector(deltalambda, temp, bj.wlambda);
    }

    /**
     * Compute the denominator part of the SPOOK equation: C = G*inv(M)*G' + eps
     * @method computeInvC
     * @param  {Number} eps
     * @return {Number}
     */
    computeC() {
        return this.computeGiMGt() + this.eps;
    }
}
