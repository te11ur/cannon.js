import {Vec3} from "../math/Vec3";
import {ConvexPolyhedron} from "./ConvexPolyhedron";

/**
 * @class Cylinder
 * @author schteppe / https://github.com/schteppe
 */
export class Cylinder extends ConvexPolyhedron {
    /**
     * @param {Number} radiusTop
     * @param {Number} radiusBottom
     * @param {Number} height
     * @param {Number} numSegments The number of segments to build the cylinder out of
     */
    constructor(radiusTop, radiusBottom, height, numSegments) {
        const N = numSegments,
            verts = [],
            axes = [],
            faces = [],
            bottomface = [],
            topface = [],
            cos = Math.cos,
            sin = Math.sin;

        // First bottom point
        verts.push(new Vec3(radiusBottom * cos(0),
            radiusBottom * sin(0),
            -height * 0.5));
        bottomface.push(0);

        // First top point
        verts.push(new Vec3(radiusTop * cos(0),
            radiusTop * sin(0),
            height * 0.5));
        topface.push(1);

        for (let i = 0; i < numSegments; i++) {
            const theta = 2 * Math.PI / N * (i + 1);
            const thetaN = 2 * Math.PI / N * (i + 0.5);
            if (i < N - 1) {
                // Bottom
                verts.push(new Vec3(radiusBottom * cos(theta),
                    radiusBottom * sin(theta),
                    -height * 0.5));
                bottomface.push(2 * i + 2);
                // Top
                verts.push(new Vec3(radiusTop * cos(theta),
                    radiusTop * sin(theta),
                    height * 0.5));
                topface.push(2 * i + 3);

                // Face
                faces.push([2 * i + 2, 2 * i + 3, 2 * i + 1, 2 * i]);
            } else {
                faces.push([0, 1, 2 * i + 1, 2 * i]); // Connect
            }

            // Axis: we can cut off half of them if we have even number of segments
            if (numSegments % 2 === 1 || i < numSegments / 2) {
                axes.push(new Vec3(cos(thetaN), sin(thetaN), 0));
            }
        }
        faces.push(topface);
        axes.push(new Vec3(0, 0, 1));

        // Reorder bottom face
        const temp = [];
        for (let i = 0; i < bottomface.length; i++) {
            temp.push(bottomface[bottomface.length - i - 1]);
        }
        faces.push(temp);

        super(verts, faces, axes);
    }
}
