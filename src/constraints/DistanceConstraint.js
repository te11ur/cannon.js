import {Constraint} from "./Constraint";
import {ContactEquation} from "../equations/ContactEquation";

/**
 * Constrains two bodies to be at a constant distance from each others center of mass.
 * @class DistanceConstraint
 * @author schteppe
 */
export class DistanceConstraint extends Constraint {

    /**
     * @property {number} distance
     */
    distance;

    /**
     * @property {ContactEquation} distanceEquation
     */
    distanceEquation;

    /**
     * @param {Body} bodyA
     * @param {Body} bodyB
     * @param {Number} [distance] The distance to keep. If undefined, it will be set to the current distance between bodyA and bodyB
     * @param {Number} [maxForce=1e6]
     */
    constructor(bodyA, bodyB, distance, maxForce = 1e6) {
        super(bodyA, bodyB);

        if (typeof distance === "undefined") {
            distance = bodyA.position.distanceTo(bodyB.position);
        }

        this.distance = distance;
        const eq = this.distanceEquation = new ContactEquation(bodyA, bodyB);
        this.equations.push(eq);

        // Make it bidirectional
        eq.minForce = -maxForce;
        eq.maxForce = maxForce;
    }

    update() {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const eq = this.distanceEquation;
        const halfDist = this.distance * 0.5;
        const normal = eq.ni;

        bodyB.position.vsub(bodyA.position, normal);
        normal.normalize();
        normal.mult(halfDist, eq.ri);
        normal.mult(-halfDist, eq.rj);
    }
}