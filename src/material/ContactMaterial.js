/**
 * Defines what happens when two materials meet.
 * @class ContactMaterial
 */
export class ContactMaterial {
    static idCounter = 0;

    /**
     * Identifier of this material
     * @property {Number} id
     */
    id = ContactMaterial.idCounter++;

    /**
     * Participating materials
     * @property {Array} materials
     * @todo  Should be .materialA and .materialB instead
     */
    materials;

    /**
     * Friction coefficient
     * @property {Number} friction
     */
    friction;

    /**
     * Restitution coefficient
     * @property {Number} restitution
     */
    restitution;

    /**
     * Stiffness of the produced contact equations
     * @property {Number} contactEquationStiffness
     */
    contactEquationStiffness;

    /**
     * Relaxation time of the produced contact equations
     * @property {Number} contactEquationRelaxation
     */
    contactEquationRelaxation;

    /**
     * Stiffness of the produced friction equations
     * @property {Number} frictionEquationStiffness
     */
    frictionEquationStiffness;

    /**
     * Relaxation time of the produced friction equations
     * @property {Number} frictionEquationRelaxation
     */
    frictionEquationRelaxation;

    /**
     * @param {Material} m1
     * @param {Material} m2
     * @param {object} [options]
     * @param {Number} [options.friction=0.3]
     * @param {Number} [options.restitution=0.3]
     * @param {number} [options.contactEquationStiffness=1e7]
     * @param {number} [options.contactEquationRelaxation=3]
     * @param {number} [options.frictionEquationStiffness=1e7]
     * @param {Number} [options.frictionEquationRelaxation=3]
     */
    constructor(m1, m2, options) {

        this.materials = [m1, m2];

        Object.assign(this, {
            friction: 0.3,
            restitution: 0.3,
            contactEquationStiffness: 1e7,
            contactEquationRelaxation: 3,
            frictionEquationStiffness: 1e7,
            frictionEquationRelaxation: 3
        }, options);
    }
}
