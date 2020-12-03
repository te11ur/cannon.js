import {Vec3} from './Vec3';
import {Quaternion} from './Quaternion';

const tmpQuat = new Quaternion();

/**
 * @class Transform
 */
export class Transform {
    options;
    /**
     * @property {Vec3} position
     */
    position;

    /**
     * @property {Quaternion} quaternion
     */
    quaternion;

    constructor(options = {}) {
        const {
            position,
            quaternion
        } = this.options = options;

        this.position = new Vec3();
        this.quaternion = new Quaternion();

        if (position) {
            this.position.copy(position);
        }

        if (quaternion) {
            this.quaternion.copy(quaternion);
        }
    }

    /**
     * @static
     * @method pointToLocaFrame
     * @param {Vec3} position
     * @param {Quaternion} quaternion
     * @param {Vec3} worldPoint
     * @param {Vec3} result
     */
    static pointToLocalFrame(position, quaternion, worldPoint, result = new Vec3()) {
        worldPoint.vsub(position, result);
        quaternion.conjugate(tmpQuat);
        tmpQuat.vmult(result, result);
        return result;
    }

    /**
     * Get a global point in local transform coordinates.
     * @method pointToLocal
     * @param  {Vec3} worldPoint
     * @param  {Vec3} result
     * @return {Vec3} The "result" vector object
     */
    pointToLocal(worldPoint, result) {
        return Transform.pointToLocalFrame(this.position, this.quaternion, worldPoint, result);
    }

    /**
     * @static
     * @method pointToWorldFrame
     * @param {Vec3} position
     * @param {Quaternion} quaternion
     * @param {Vec3} localPoint
     * @param {Vec3} result
     */
    static pointToWorldFrame(position, quaternion, localPoint, result = new Vec3()) {
        quaternion.vmult(localPoint, result);
        result.vadd(position, result);
        return result;
    }

    /**
     * Get a local point in global transform coordinates.
     * @method pointToWorld
     * @param  {Vec3} localPoint
     * @param  {Vec3} result
     * @return {Vec3} The "result" vector object
     */
    pointToWorld(localPoint, result) {
        return Transform.pointToWorldFrame(this.position, this.quaternion, localPoint, result);
    }

    vectorToWorldFrame(localVector, result = new Vec3()) {
        this.quaternion.vmult(localVector, result);
        return result;
    }

    static vectorToWorldFrame(quaternion, localVector, result) {
        quaternion.vmult(localVector, result);
        return result;
    }

    static vectorToLocalFrame(position, quaternion, worldVector, result = new Vec3()) {
        quaternion.w *= -1;
        quaternion.vmult(worldVector, result);
        quaternion.w *= -1;
        return result;
    }
}