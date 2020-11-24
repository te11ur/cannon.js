import {PointToPointConstraint} from './PointToPointConstraint';
import {ConeEquation} from '../equations/ConeEquation';
import {RotationalEquation} from '../equations/RotationalEquation';
import {Vec3} from '../math/Vec3';

/**
 * @class ConeTwistConstraint
 * @author schteppe
 */
export class ConeTwistConstraint extends PointToPointConstraint {
    axisA;
    axisB;

    /**
     * @property {ConeEquation} coneEquation
     */
    coneEquation;

    /**
     * @property {RotationalEquation} twistEquation
     */
    twistEquation;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {object} [options]
     * @param {Vec3} [options.pivotA]
     * @param {Vec3} [options.pivotB]
     * @param {Vec3} [options.axisA]
     * @param {Vec3} [options.axisB]
     * @param {Number} [options.maxForce=1e6]
     */
    constructor(bodyA, bodyB, options) {
        let {
            maxForce = 1e6,
            pivotA,
            pivotB,
            axisA,
            axisB,
            collideConnected,
            angle = 0,
            twistAngle = 0
        } = Object.assign({}, options);

        // Set pivot point in between
        pivotA = pivotA ? pivotA.clone() : new Vec3();
        pivotB = pivotB ? pivotB.clone() : new Vec3();
        axisA = axisA ? axisA.clone() : new Vec3();
        axisB = axisB ? axisB.clone() : new Vec3();

        super(bodyA, pivotA, bodyB, pivotB, maxForce);

        this.axisA = axisA;
        this.axisB = axisB;
        this.collideConnected = !!collideConnected;
        this.angle = angle;

        const c = this.coneEquation = new ConeEquation(bodyA, bodyB, options);
        const t = this.twistEquation = new RotationalEquation(bodyA, bodyB, options);
        this.twistAngle = twistAngle;

        // Make the cone equation push the bodies toward the cone axis, not outward
        c.maxForce = 0;
        c.minForce = -maxForce;

        // Make the twist equation add torque toward the initial position
        t.maxForce = 0;
        t.minForce = -maxForce;

        this.equations.push(c, t);
    }

    update() {
        const bodyA = this.bodyA,
            bodyB = this.bodyB,
            cone = this.coneEquation,
            twist = this.twistEquation;

        super.update();

        // Update the axes to the cone constraint
        bodyA.vectorToWorldFrame(this.axisA, cone.axisA);
        bodyB.vectorToWorldFrame(this.axisB, cone.axisB);

        // Update the world axes in the twist constraint
        this.axisA.tangents(twist.axisA, twist.axisA);
        bodyA.vectorToWorldFrame(twist.axisA, twist.axisA);

        this.axisB.tangents(twist.axisB, twist.axisB);
        bodyB.vectorToWorldFrame(twist.axisB, twist.axisB);

        cone.angle = this.angle;
        twist.maxAngle = this.twistAngle;
    }
}
