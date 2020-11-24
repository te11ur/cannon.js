/**
 * Constraint base class
 * @class Constraint
 * @author schteppe
 */
export class Constraint {
    static idCounter = 0;
    /**
     * Equations to be solved in this constraint
     * @property equations
     * @type {Array}
     */
    equations;

    /**
     * @property {Body} bodyA
     */
    bodyA;

    /**
     * @property {Body} bodyB
     */
    bodyB;

    /**
     * @property {Number} id
     */
    id = Constraint.idCounter++;

    /**
     * Set to true if you want the bodies to collide when they are connected.
     * @property collideConnected
     * @type {boolean}
     */
    collideConnected;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {object} [options]
     * @param {boolean} [options.collideConnected=true]
     * @param {boolean} [options.wakeUpBodies=true]
     */
    constructor(bodyA, bodyB, options) {
        const {
            collideConnected = true,
            wakeUpBodies = true,
        } = Object.assign({}, options);

        this.equations = [];
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.collideConnected = collideConnected;
        this.wakeUpBodies = wakeUpBodies;

        if (wakeUpBodies) {
            if (bodyA) {
                bodyA.wakeUp();
            }
            if (bodyB) {
                bodyB.wakeUp();
            }
        }
    }

    /**
     * Update all the equations with data.
     * @method update
     */
    update() {
        throw new Error("method update() not implmemented in this Constraint subclass!");
    }

    /**
     * Enables all equations in the constraint.
     * @method enable
     */
    enable() {
        const eqs = this.equations;
        for (let i = 0; i < eqs.length; i++) {
            eqs[i].enabled = true;
        }
    }

    /**
     * Disables all equations in the constraint.
     * @method disable
     */
    disable() {
        const eqs = this.equations;
        for (let i = 0; i < eqs.length; i++) {
            eqs[i].enabled = false;
        }
    }
}
