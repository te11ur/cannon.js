import {AABB} from '../collision/AABB';
import {Vec3} from '../math/Vec3';

/**
 * @class OctreeNode
 */
export class OctreeNode {
    options;

    /**
     * The root node
     * @property {OctreeNode} root
     */
    root;

    /**
     * Boundary of this node
     * @property {AABB} aabb
     */
    aabb;

    /**
     * Contained data at the current node level.
     * @property {Array} data
     */
    data = [];

    /**
     * Children to this node
     * @property {Array} children
     */
    children = [];

    /**
     * @param {object} [options]
     * @param {Octree} [options.root]
     * @param {AABB} [options.aabb]
     */
    constructor(options) {
        const {
            root = null,
            aabb,
        } = this.options = Object.assign({}, options);

        this.root = root;
        this.aabb = new AABB();

        if (aabb) {
            this.aabb.copy(aabb);
        }
    }
}

const halfDiagonal = new Vec3();
const tmpAABB = new AABB();

/**
 * @class Octree
 * @extends OctreeNode
 */
export class Octree extends OctreeNode {

    /**
     * Maximum subdivision depth
     * @property {number} maxDepth
     */
    maxDepth;

    /**
     * @param {AABB} aabb The total AABB of the tree
     * @param {object} [options]
     * @param {number} [options.maxDepth=8]
     */
    constructor(aabb, options) {
        options = Object.assign({
            root: null,
            aabb: aabb
        }, options);

        const {
            maxDepth = 8
        } = options;

        super(options);

        this.maxDepth = maxDepth;
    }

    reset(aabb, options) {
        this.children.length = this.data.length = 0;
    }

    /**
     * Insert data into this node
     * @method insert
     * @param  {AABB} aabb
     * @param  {object} elementData
     * @return {boolean} True if successful, otherwise false
     */
    insert(aabb, elementData, level) {
        const nodeData = this.data;
        level = level || 0;

        // Ignore objects that do not belong in this node
        if (!this.aabb.contains(aabb)) {
            return false; // object cannot be added
        }

        const children = this.children;

        if (level < (this.maxDepth || this.root.maxDepth)) {
            // Subdivide if there are no children yet
            let subdivided = false;
            if (!children.length) {
                this.subdivide();
                subdivided = true;
            }

            // add to whichever node will accept it
            for (let i = 0; i !== 8; i++) {
                if (children[i].insert(aabb, elementData, level + 1)) {
                    return true;
                }
            }

            if (subdivided) {
                // No children accepted! Might as well just remove em since they contain none
                children.length = 0;
            }
        }

        // Too deep, or children didnt want it. add it in current node
        nodeData.push(elementData);

        return true;
    }

    /**
     * Create 8 equally sized children nodes and put them in the .children array.
     * @method subdivide
     */
    subdivide() {
        const aabb = this.aabb;
        const l = aabb.lowerBound;
        const u = aabb.upperBound;

        const children = this.children;

        children.push(
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(0, 0, 0)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(1, 0, 0)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(1, 1, 0)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(1, 1, 1)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(0, 1, 1)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(0, 0, 1)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(1, 0, 1)})}),
            new OctreeNode({aabb: new AABB({lowerBound: new Vec3(0, 1, 0)})})
        );

        u.vsub(l, halfDiagonal);
        halfDiagonal.scale(0.5, halfDiagonal);

        const root = this.root || this;

        for (let i = 0; i !== 8; i++) {
            const child = children[i];

            // Set current node as root
            child.root = root;

            // Compute bounds
            const lowerBound = child.aabb.lowerBound;
            lowerBound.x *= halfDiagonal.x;
            lowerBound.y *= halfDiagonal.y;
            lowerBound.z *= halfDiagonal.z;

            lowerBound.vadd(l, lowerBound);

            // Upper bound is always lower bound + halfDiagonal
            lowerBound.vadd(halfDiagonal, child.aabb.upperBound);
        }
    }

    /**
     * Get all data, potentially within an AABB
     * @method aabbQuery
     * @param  {AABB} aabb
     * @param  {array} result
     * @return {array} The "result" object
     */
    aabbQuery(aabb, result) {

        const nodeData = this.data;

        // abort if the range does not intersect this node
        // if (!this.aabb.overlaps(aabb)){
        //     return result;
        // }

        // Add objects at this level
        // Array.prototype.push.apply(result, nodeData);

        // Add child data
        // @todo unwrap recursion into a queue / loop, that's faster in JS
        const children = this.children;


        // for (let i = 0, N = this.children.length; i !== N; i++) {
        //     children[i].aabbQuery(aabb, result);
        // }

        const queue = [this];
        while (queue.length) {
            const node = queue.pop();
            if (node.aabb.overlaps(aabb)) {
                Array.prototype.push.apply(result, node.data);
            }
            Array.prototype.push.apply(queue, node.children);
        }

        return result;
    }

    /**
     * Get all data, potentially intersected by a ray.
     * @method rayQuery
     * @param  {Ray} ray
     * @param  {Transform} treeTransform
     * @param  {array} result
     * @return {array} The "result" object
     */
    rayQuery(ray, treeTransform, result) {
        // Use aabb query for now.
        // @todo implement real ray query which needs less lookups
        ray.getAABB(tmpAABB);
        tmpAABB.toLocalFrame(treeTransform, tmpAABB);
        this.aabbQuery(tmpAABB, result);

        return result;
    }

    /**
     * @method removeEmptyNodes
     */
    removeEmptyNodes() {
        const queue = [this];
        while (queue.length) {
            const node = queue.pop();
            for (let i = node.children.length - 1; i >= 0; i--) {
                if (!node.children[i].data.length) {
                    node.children.splice(i, 1);
                }
            }
            Array.prototype.push.apply(queue, node.children);
        }
    }
}
