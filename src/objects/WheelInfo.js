import {Vec3} from '../math/Vec3';
import {Transform} from '../math/Transform';
import {RaycastResult} from '../collision/RaycastResult';

const chassis_velocity_at_contactPoint = new Vec3();
const relpos = new Vec3();

/**
 * @class WheelInfo
 */
export class WheelInfo {

    /**
     * Max travel distance of the suspension, in meters.
     * @property {number} maxSuspensionTravel
     */
    maxSuspensionTravel;

    /**
     * Speed to apply to the wheel rotation when the wheel is sliding.
     * @property {number} customSlidingRotationalSpeed
     */
    customSlidingRotationalSpeed;

    /**
     * If the customSlidingRotationalSpeed should be used.
     * @property {Boolean} useCustomSlidingRotationalSpeed
     */
    useCustomSlidingRotationalSpeed;

    /**
     * @property {Boolean} sliding
     */
    sliding = false;

    /**
     * Connection point, defined locally in the chassis body frame.
     * @property {Vec3} chassisConnectionPointLocal
     */
    chassisConnectionPointLocal;

    /**
     * @property {Vec3} chassisConnectionPointWorld
     */
    chassisConnectionPointWorld;

    /**
     * @property {Vec3} directionLocal
     */
    directionLocal;

    /**
     * @property {Vec3} directionWorld
     */
    directionWorld;

    /**
     * @property {Vec3} axleLocal
     */
    axleLocal;

    /**
     * @property {Vec3} axleWorld
     */
    axleWorld;

    /**
     * @property {number} suspensionRestLength
     */
    suspensionRestLength;

    /**
     * @property {number} suspensionMaxLength
     */
    suspensionMaxLength;

    /**
     * @property {number} radius
     */
    radius;

    /**
     * @property {number} suspensionStiffness
     */
    suspensionStiffness;

    /**
     * @property {number} dampingCompression
     */
    dampingCompression;

    /**
     * @property {number} dampingRelaxation
     */
    dampingRelaxation;

    /**
     * @property {number} frictionSlip
     */
    frictionSlip;

    /**
     * @property {number} steering
     */
    steering;

    /**
     * Rotation value, in radians.
     * @property {number} rotation
     */
    rotation;

    /**
     * @property {number} deltaRotation
     */
    deltaRotation;

    /**
     * @property {number} rollInfluence
     */
    rollInfluence;

    /**
     * @property {number} maxSuspensionForce
     */
    maxSuspensionForce;

    /**
     * @property {number} engineForce
     */
    engineForce = 0;

    /**
     * @property {number} brake
     */
    brake = 0;

    /**
     * @property {number} isFrontWheel
     */
    isFrontWheel;

    /**
     * @property {number} clippedInvContactDotSuspension
     */
    clippedInvContactDotSuspension;

    /**
     * @property {number} suspensionRelativeVelocity
     */
    suspensionRelativeVelocity;

    /**
     * @property {number} suspensionForce
     */
    suspensionForce;

    /**
     * @property {number} skidInfo
     */
    skidInfo;

    /**
     * @property {number} suspensionLength
     */
    suspensionLength;

    /**
     * @property {number} sideImpulse
     */
    sideImpulse = 0;

    /**
     * @property {number} forwardImpulse
     */
    forwardImpulse = 0;

    /**
     * The result from raycasting
     * @property {RaycastResult} raycastResult
     */
    raycastResult = new RaycastResult();

    /**
     * Wheel world transform
     * @property {Transform} worldTransform
     */
    worldTransform = new Transform();

    /**
     * @property {boolean} isInContact
     */
    isInContact = false;

    /**
     * @param {Object} [options]
     * @param {Vec3} [options.chassisConnectionPointLocal]
     * @param {Vec3} [options.chassisConnectionPointWorld]
     * @param {Vec3} [options.directionLocal]
     * @param {Vec3} [options.directionWorld]
     * @param {Vec3} [options.axleLocal]
     * @param {Vec3} [options.axleWorld]
     * @param {number} [options.suspensionRestLength=1]
     * @param {number} [options.suspensionMaxLength=2]
     * @param {number} [options.radius=1]
     * @param {number} [options.suspensionStiffness=100]
     * @param {number} [options.dampingCompression=10]
     * @param {number} [options.dampingRelaxation=10]
     * @param {number} [options.frictionSlip=10000]
     * @param {number} [options.steering=0]
     * @param {number} [options.rotation=0]
     * @param {number} [options.deltaRotation=0]
     * @param {number} [options.rollInfluence=0.01]
     * @param {number} [options.maxSuspensionForce]
     * @param {boolean} [options.isFrontWheel=true]
     * @param {number} [options.clippedInvContactDotSuspension=1]
     * @param {number} [options.suspensionRelativeVelocity=0]
     * @param {number} [options.suspensionForce=0]
     * @param {number} [options.skidInfo=0]
     * @param {number} [options.suspensionLength=0]
     * @param {number} [options.maxSuspensionTravel=1]
     * @param {boolean} [options.useCustomSlidingRotationalSpeed=false]
     * @param {number} [options.customSlidingRotationalSpeed=-0.1]
     */
    constructor(options = {}) {
        const {
            chassisConnectionPointLocal,
            chassisConnectionPointWorld,
            directionLocal,
            directionWorld,
            axleLocal,
            axleWorld
        } = Object.assign(this, {
            suspensionRestLength: 1,
            suspensionMaxLength: 2,
            radius: 1,
            suspensionStiffness: 100,
            dampingCompression: 10,
            dampingRelaxation: 10,
            frictionSlip: 10000,
            steering: 0,
            rotation: 0,
            deltaRotation: 0,
            rollInfluence: 0.01,
            maxSuspensionForce: Number.MAX_VALUE,
            isFrontWheel: true,
            clippedInvContactDotSuspension: 1,
            suspensionRelativeVelocity: 0,
            suspensionForce: 0,
            skidInfo: 0,
            suspensionLength: 0,
            maxSuspensionTravel: 1,
            useCustomSlidingRotationalSpeed: false,
            customSlidingRotationalSpeed: -0.1
        }, options);

        this.chassisConnectionPointLocal = new Vec3();
        this.chassisConnectionPointWorld = new Vec3();
        this.directionLocal = new Vec3();
        this.directionWorld = new Vec3();
        this.axleLocal = new Vec3();
        this.axleWorld = new Vec3();

        if(chassisConnectionPointLocal) {
            this.chassisConnectionPointLocal.copy(chassisConnectionPointLocal);
        }

        if(chassisConnectionPointWorld) {
            this.chassisConnectionPointWorld.copy(chassisConnectionPointWorld);
        }

        if(directionLocal) {
            this.directionLocal.copy(directionLocal);
        }

        if(directionWorld) {
            this.directionWorld.copy(directionWorld);
        }

        if(axleLocal) {
            this.axleLocal.copy(axleLocal);
        }

        if(axleWorld) {
            this.axleWorld.copy(axleWorld);
        }
    }

    updateWheel(chassis) {
        const raycastResult = this.raycastResult;

        if (this.isInContact) {
            const project = raycastResult.hitNormalWorld.dot(raycastResult.directionWorld);
            raycastResult.hitPointWorld.vsub(chassis.position, relpos);
            chassis.getVelocityAtWorldPoint(relpos, chassis_velocity_at_contactPoint);
            const projVel = raycastResult.hitNormalWorld.dot(chassis_velocity_at_contactPoint);
            if (project >= -0.1) {
                this.suspensionRelativeVelocity = 0.0;
                this.clippedInvContactDotSuspension = 1.0 / 0.1;
            } else {
                const inv = -1 / project;
                this.suspensionRelativeVelocity = projVel * inv;
                this.clippedInvContactDotSuspension = inv;
            }

        } else {
            // Not in contact : position wheel in a nice (rest length) position
            raycastResult.suspensionLength = this.suspensionRestLength;
            this.suspensionRelativeVelocity = 0.0;
            raycastResult.directionWorld.scale(-1, raycastResult.hitNormalWorld);
            this.clippedInvContactDotSuspension = 1.0;
        }
    }
}