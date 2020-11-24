/**
 * Defines a physics material.
 * @class Material
 * @author schteppe
 */
export class Material {
    static idCounter = 0;

    /**
     * @property name
     * @type {String}
     */
    name = '';

    /**
     * material id.
     * @property id
     * @type {number}
     */
    id = Material.idCounter++;

    /**
     * Friction for this material. If non-negative, it will be used instead of the friction given by ContactMaterials. If there's no matching ContactMaterial, the value from .defaultContactMaterial in the World will be used.
     * @property {number} friction
     */
    friction;

    /**
     * Restitution for this material. If non-negative, it will be used instead of the restitution given by ContactMaterials. If there's no matching ContactMaterial, the value from .defaultContactMaterial in the World will be used.
     * @property {number} restitution
     */
    restitution;

    /**
     * @param {object} [options]
     */
    constructor(options) {
        // Backwards compatibility fix
        if (typeof options === 'string') {
            this.name = options;
            this.restitution = -1;
            this.friction = -1;
        } else {
            const {
                restitution = -1,
                friction = -1
            } = Object.assign({}, options);

            this.restitution = restitution;
            this.friction = friction;
        }
    }
}
