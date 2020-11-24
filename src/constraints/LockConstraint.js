import {PointToPointConstraint} from "./PointToPointConstraint";
import {RotationalEquation} from "../equations/RotationalEquation";
import {Vec3} from "../math/Vec3";

/**
 * Lock constraint. Will remove all degrees of freedom between the bodies.
 * @class LockConstraint
 * @author schteppe
 */
export class LockConstraint extends PointToPointConstraint {
    xA;
    xB;
    yA;
    yB;
    zA;
    zB;

    /**
     * @property {RotationalEquation} rotationalEquation1
     */
    rotationalEquation1;

    /**
     * @property {RotationalEquation} rotationalEquation2
     */
    rotationalEquation2;

    /**
     * @property {RotationalEquation} rotationalEquation3
     */
    rotationalEquation3;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {object} [options]
     * @param {Number} [options.maxForce=1e6]
     */
    constructor(bodyA, bodyB, options) {
        const {
            maxForce = 1e6
        } = Object.assign({}, options);

        // Set pivot point in between
        const pivotA = new Vec3();
        const pivotB = new Vec3();
        const halfWay = new Vec3();
        bodyA.position.vadd(bodyB.position, halfWay);
        halfWay.scale(0.5, halfWay);
        bodyB.pointToLocalFrame(halfWay, pivotB);
        bodyA.pointToLocalFrame(halfWay, pivotA);

        // The point-to-point constraint will keep a point shared between the bodies
        super(bodyA, pivotA, bodyB, pivotB, maxForce);

        // Store initial rotation of the bodies as unit vectors in the local body spaces
        this.xA = bodyA.vectorToLocalFrame(Vec3.UNIT_X);
        this.xB = bodyB.vectorToLocalFrame(Vec3.UNIT_X);
        this.yA = bodyA.vectorToLocalFrame(Vec3.UNIT_Y);
        this.yB = bodyB.vectorToLocalFrame(Vec3.UNIT_Y);
        this.zA = bodyA.vectorToLocalFrame(Vec3.UNIT_Z);
        this.zB = bodyB.vectorToLocalFrame(Vec3.UNIT_Z);

        // ...and the following rotational equations will keep all rotational DOF's in place

        const r1 = this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, options);
        const r2 = this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, options);
        const r3 = this.rotationalEquation3 = new RotationalEquation(bodyA, bodyB, options);

        this.equations.push(r1, r2, r3);
    }

    update() {
        const bodyA = this.bodyA,
            bodyB = this.bodyB,
            r1 = this.rotationalEquation1,
            r2 = this.rotationalEquation2,
            r3 = this.rotationalEquation3;

        super.update();

        // These vector pairs must be orthogonal
        bodyA.vectorToWorldFrame(this.xA, r1.axisA);
        bodyB.vectorToWorldFrame(this.yB, r1.axisB);

        bodyA.vectorToWorldFrame(this.yA, r2.axisA);
        bodyB.vectorToWorldFrame(this.zB, r2.axisB);

        bodyA.vectorToWorldFrame(this.zA, r3.axisA);
        bodyB.vectorToWorldFrame(this.xB, r3.axisB);
    }
}

