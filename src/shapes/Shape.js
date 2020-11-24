import {Vec3} from '../math/Vec3';
import {Material} from '../material/Material';

/**
 * Base class for shapes
 * @class Shape
 * @author schteppe
 */
export class Shape {
    static idCounter = 0;

    /**
     * The available shape types.
     * @static
     * @property types
     * @type {Object}
     */
    static types = {
        SPHERE: 1,
        PLANE: 2,
        BOX: 4,
        COMPOUND: 8,
        CONVEXPOLYHEDRON: 16,
        HEIGHTFIELD: 32,
        PARTICLE: 64,
        CYLINDER: 128,
        TRIMESH: 256
    };

    /**
     * Identifyer of the Shape.
     * @property {number} id
     */
    id = Shape.idCounter++;

    /**
     * The type of this shape. Must be set to an int > 0 by subclasses.
     * @property type
     * @type {Number}
     * @see Shape.types
     */
    type;

    /**
     * The local bounding sphere radius of this shape.
     * @property {Number} boundingSphereRadius
     */
    boundingSphereRadius;

    /**
     * Whether to produce contact forces when in contact with other bodies. Note that contacts will be generated, but they will be disabled.
     * @property {boolean} collisionResponse
     */
    collisionResponse;

    /**
     * @property {Number} collisionFilterGroup
     */
    collisionFilterGroup;

    /**
     * @property {Number} collisionFilterMask
     */
    collisionFilterMask;

    /**
     * @property {Material} material
     */
    material;

    /**
     * @property {Body} body
     */
    body;

    /**
     * @param {object} [options]
     * @param {number} [options.collisionFilterGroup=1]
     * @param {number} [options.collisionFilterMask=-1]
     * @param {number} [options.collisionResponse=true]
     * @param {number} [options.material=null]
     */
    constructor(options) {
        const {
            type = 0,
            collisionFilterGroup = 1,
            collisionFilterMask = -1,
            collisionResponse = true,
            material = null
        } = Object.assign({}, options);

        this.type = type;
        this.boundingSphereRadius = 0;
        this.collisionFilterGroup = collisionFilterGroup;
        this.collisionFilterMask = collisionFilterMask;
        this.collisionResponse = collisionResponse;
        this.material = material;
        this.body = null;
    }

    /**
     * Computes the bounding sphere radius. The result is stored in the property .boundingSphereRadius
     * @method updateBoundingSphereRadius
     */
    updateBoundingSphereRadius() {
        throw "computeBoundingSphereRadius() not implemented for shape type " + this.type;
    }

    /**
     * Get the volume of this shape
     * @method volume
     * @return {Number}
     */
    volume() {
        throw "volume() not implemented for shape type " + this.type;
    }

    /**
     * Calculates the inertia in the local frame for this shape.
     * @method calculateLocalInertia
     * @param {Number} mass
     * @param {Vec3} target
     * @see http://en.wikipedia.org/wiki/List_of_moments_of_inertia
     */
    calculateLocalInertia(mass, target) {
        throw "calculateLocalInertia() not implemented for shape type " + this.type;
    }
}

