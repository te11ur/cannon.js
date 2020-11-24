import {Pool} from "./Pool";
import {Vec3} from "../math/Vec3";

/**
 * @class Vec3Pool
 * @extends Pool
 */
export class Vec3Pool extends Pool {
    type = Vec3;
    /**
     * Construct a vector
     * @method constructObject
     * @return {Vec3}
     */
    constructObject (){
        return new Vec3();
    }
}

