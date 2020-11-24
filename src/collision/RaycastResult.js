import {Vec3} from '../math/Vec3';

/**
 * Storage for Ray casting data.
 * @class RaycastResult
 * @constructor
 */
export class RaycastResult {

    /**
     * @property {Vec3} rayFromWorld
     */
    rayFromWorld = new Vec3();

    /**
     * @property {Vec3} rayToWorld
     */
    rayToWorld = new Vec3();

    /**
     * @property {Vec3} hitNormalWorld
     */
    hitNormalWorld = new Vec3();

    /**
     * @property {Vec3} hitPointWorld
     */
    hitPointWorld = new Vec3();

    /**
     * @property {boolean} hasHit
     */
    hasHit = false;

    /**
     * The hit shape, or null.
     * @property {Shape} shape
     */
    shape = null;

    /**
     * The hit body, or null.
     * @property {Body} body
     */
    body = null;

    /**
     * The index of the hit triangle, if the hit shape was a trimesh.
     * @property {number} hitFaceIndex
     * @default -1
     */
    hitFaceIndex = -1;

    /**
     * Distance to the hit. Will be set to -1 if there was no hit.
     * @property {number} distance
     * @default -1
     */
    distance = -1;

    /**
     * If the ray should stop traversing the bodies.
     * @private
     * @property {Boolean} _shouldStop
     * @default false
     */
    _shouldStop = false;

    /**
     * Reset all result data.
     * @method reset
     */
    reset() {
        this.rayFromWorld.setZero();
        this.rayToWorld.setZero();
        this.hitNormalWorld.setZero();
        this.hitPointWorld.setZero();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this._shouldStop = false;
    }

    /**
     * @method abort
     */
    abort() {
        this._shouldStop = true;
    }

    /**
     * @method set
     * @param {Vec3} rayFromWorld
     * @param {Vec3} rayToWorld
     * @param {Vec3} hitNormalWorld
     * @param {Vec3} hitPointWorld
     * @param {Shape} shape
     * @param {Body} body
     * @param {number} distance
     */
    set(
        rayFromWorld,
        rayToWorld,
        hitNormalWorld,
        hitPointWorld,
        shape,
        body,
        distance
    ) {
        this.rayFromWorld.copy(rayFromWorld);
        this.rayToWorld.copy(rayToWorld);
        this.hitNormalWorld.copy(hitNormalWorld);
        this.hitPointWorld.copy(hitPointWorld);
        this.shape = shape;
        this.body = body;
        this.distance = distance;
    }
}