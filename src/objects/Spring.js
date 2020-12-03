import {Vec3} from '../math/Vec3';

const applyForce_r = new Vec3();
const applyForce_r_unit = new Vec3();
const applyForce_u = new Vec3();
const applyForce_f = new Vec3();
const applyForce_worldAnchorA = new Vec3();
const applyForce_worldAnchorB = new Vec3();
const applyForce_ri = new Vec3();
const applyForce_rj = new Vec3();
const applyForce_ri_x_f = new Vec3();
const applyForce_rj_x_f = new Vec3();
const applyForce_tmp = new Vec3();

/**
 * A spring, connecting two bodies.
 * @class Spring
 */
export class Spring {
    /**
     * Rest length of the spring.
     * @property restLength
     * @type {number}
     */
    restLength;

    /**
     * Stiffness of the spring.
     * @property stiffness
     * @type {number}
     */
    stiffness;

    /**
     * Damping of the spring.
     * @property damping
     * @type {number}
     */
    damping;

    /**
     * First connected body.
     * @property bodyA
     * @type {Body}
     */
    bodyA;

    /**
     * Second connected body.
     * @property bodyB
     * @type {Body}
     */
    bodyB;

    /**
     * Anchor for bodyA in local bodyA coordinates.
     * @property localAnchorA
     * @type {Vec3}
     */
    localAnchorA;

    /**
     * Anchor for bodyB in local bodyB coordinates.
     * @property localAnchorB
     * @type {Vec3}
     */
    localAnchorB;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Object} [options]
     * @param {number} [options.restLength]   A number > 0. Default: 1
     * @param {number} [options.stiffness]    A number >= 0. Default: 100
     * @param {number} [options.damping]      A number >= 0. Default: 1
     * @param {Vec3}  [options.worldAnchorA] Where to hook the spring to body A, in world coordinates.
     * @param {Vec3}  [options.worldAnchorB]
     * @param {Vec3}  [options.localAnchorA] Where to hook the spring to body A, in local body coordinates.
     * @param {Vec3}  [options.localAnchorB]
     */
    constructor(bodyA, bodyB, options = {}) {
        this.bodyA = bodyA;
        this.bodyB = bodyB;

        this.localAnchorA = new Vec3();
        this.localAnchorB = new Vec3();
        this.stiffness = 100;
        this.damping = 1;

        const {
            localAnchorA,
            localAnchorB,
            worldAnchorA,
            worldAnchorB,
            restLength,
            stiffness,
            damping
        } = options;

        if (typeof restLength === "number") {
            this.restLength = restLength;
        }

        if (stiffness) {
            this.stiffness = stiffness;
        }

        if (damping) {
            this.damping = damping;
        }

        if (localAnchorA) {
            this.localAnchorA.copy(localAnchorA);
        }

        if (localAnchorB) {
            this.localAnchorB.copy(localAnchorB);
        }

        if (worldAnchorA) {
            this.setWorldAnchorA(worldAnchorA);
        }

        if (worldAnchorB) {
            this.setWorldAnchorB(worldAnchorB);
        }
    }

    /**
     * Set the anchor point on body A, using world coordinates.
     * @method setWorldAnchorA
     * @param {Vec3} worldAnchorA
     */
    setWorldAnchorA(worldAnchorA) {
        this.bodyA.pointToLocalFrame(worldAnchorA, this.localAnchorA);
    }

    /**
     * Set the anchor point on body B, using world coordinates.
     * @method setWorldAnchorB
     * @param {Vec3} worldAnchorB
     */
    setWorldAnchorB(worldAnchorB) {
        this.bodyB.pointToLocalFrame(worldAnchorB, this.localAnchorB);
    }

    /**
     * Get the anchor point on body A, in world coordinates.
     * @method getWorldAnchorA
     * @param {Vec3} result The vector to store the result in.
     */
    getWorldAnchorA(result) {
        this.bodyA.pointToWorldFrame(this.localAnchorA, result);
    }

    /**
     * Get the anchor point on body B, in world coordinates.
     * @method getWorldAnchorB
     * @param {Vec3} result The vector to store the result in.
     */
    getWorldAnchorB(result) {
        this.bodyB.pointToWorldFrame(this.localAnchorB, result);
    }

    /**
     * Apply the spring force to the connected bodies.
     * @method applyForce
     */
    applyForce() {
        const k = this.stiffness,
            d = this.damping,
            l = this.restLength,
            bodyA = this.bodyA,
            bodyB = this.bodyB,
            r = applyForce_r,
            r_unit = applyForce_r_unit,
            u = applyForce_u,
            f = applyForce_f,
            tmp = applyForce_tmp;

        const worldAnchorA = applyForce_worldAnchorA,
            worldAnchorB = applyForce_worldAnchorB,
            ri = applyForce_ri,
            rj = applyForce_rj,
            ri_x_f = applyForce_ri_x_f,
            rj_x_f = applyForce_rj_x_f;

        // Get world anchors
        this.getWorldAnchorA(worldAnchorA);
        this.getWorldAnchorB(worldAnchorB);

        // Get offset points
        worldAnchorA.vsub(bodyA.position, ri);
        worldAnchorB.vsub(bodyB.position, rj);

        // Compute distance vector between world anchor points
        worldAnchorB.vsub(worldAnchorA, r);
        const rlen = r.norm();
        r_unit.copy(r);
        r_unit.normalize();

        // Compute relative velocity of the anchor points, u
        bodyB.velocity.vsub(bodyA.velocity, u);
        // Add rotational velocity

        bodyB.angularVelocity.cross(rj, tmp);
        u.vadd(tmp, u);
        bodyA.angularVelocity.cross(ri, tmp);
        u.vsub(tmp, u);

        // F = - k * ( x - L ) - D * ( u )
        r_unit.mult(-k * (rlen - l) - d * u.dot(r_unit), f);

        // Add forces to bodies
        bodyA.force.vsub(f, bodyA.force);
        bodyB.force.vadd(f, bodyB.force);

        // Angular force
        ri.cross(f, ri_x_f);
        rj.cross(f, rj_x_f);
        bodyA.torque.vsub(ri_x_f, bodyA.torque);
        bodyB.torque.vadd(rj_x_f, bodyB.torque);
    }
}