import {Vec3} from "../math/Vec3";
import {Quaternion} from "../math/Quaternion";
import {GSSolver} from "../solver/GSSolver";
import {Body} from "../objects/Body";
import {Narrowphase} from "./Narrowphase";
import {EventTarget} from "../utils/EventTarget";
import {ArrayCollisionMatrix} from "../collision/ArrayCollisionMatrix";
import {OverlapKeeper} from "../collision/OverlapKeeper";
import {Material} from "../material/Material";
import {ContactMaterial} from "../material/ContactMaterial";
import {TupleDictionary} from "../utils/TupleDictionary";
import {RaycastResult} from "../collision/RaycastResult";
import {Ray} from "../collision/Ray";
import {NaiveBroadphase} from "../collision/NaiveBroadphase";

// Temp stuff
const tmpRay = new Ray();

/**
 * Dispatched after the world has stepped forward in time.
 * @event postStep
 */
const World_step_postStepEvent = {type: "postStep"}; // Reusable event objects to save memory
/**
 * Dispatched before the world steps forward in time.
 * @event preStep
 */
const World_step_preStepEvent = {type: "preStep"};
const World_step_collideEvent = {type: Body.COLLIDE_EVENT_NAME, body: null, contact: null};
const World_step_oldContacts = []; // Pools for unused objects
const World_step_frictionEquationPool = [];
const World_step_p1 = []; // Reusable arrays for collision pairs
const World_step_p2 = [];

const additions = [];
const removals = [];

const beginContactEvent = {
    type: 'beginContact',
    bodyA: null,
    bodyB: null
};

const endContactEvent = {
    type: 'endContact',
    bodyA: null,
    bodyB: null
};

const beginShapeContactEvent = {
    type: 'beginShapeContact',
    bodyA: null,
    bodyB: null,
    shapeA: null,
    shapeB: null
};

const endShapeContactEvent = {
    type: 'endShapeContact',
    bodyA: null,
    bodyB: null,
    shapeA: null,
    shapeB: null
};

/**
 * The physics world
 * @class World
 */
export class World extends EventTarget {
    /**
     * Currently / last used timestep. Is set to -1 if not available. This value is updated before each internal step, which means that it is "fresh" inside event callbacks.
     * @property {Number} dt
     */
    dt;

    /**
     * Makes bodies go to sleep when they've been inactive
     * @property allowSleep
     * @type {Boolean}
     * @default false
     */
    allowSleep;

    /**
     * All the current contacts (instances of ContactEquation) in the world.
     * @property contacts
     * @type {Array}
     */
    contacts;
    frictionEquations;

    /**
     * How often to normalize quaternions. Set to 0 for every step, 1 for every second etc.. A larger value increases performance. If bodies tend to explode, set to a smaller value (zero to be sure nothing can go wrong).
     * @property quatNormalizeSkip
     * @type {Number}
     * @default 0
     */
    quatNormalizeSkip;

    /**
     * Set to true to use fast quaternion normalization. It is often enough accurate to use. If bodies tend to explode, set to false.
     * @property quatNormalizeFast
     * @type {Boolean}
     * @see Quaternion.normalizeFast
     * @see Quaternion.normalize
     * @default false
     */
    quatNormalizeFast;

    /**
     * The wall-clock time since simulation start
     * @property time
     * @type {Number}
     */
    time;

    /**
     * Number of timesteps taken since start
     * @property stepnumber
     * @type {Number}
     */
    stepnumber;

    default_dt;

    nextId;
    /**
     * @property gravity
     * @type {Vec3}
     */
    gravity;

    /**
     * The broadphase algorithm to use. Default is NaiveBroadphase
     * @property broadphase
     * @type {Broadphase}
     */
    broadphase;

    /**
     * @property bodies
     * @type {Array}
     */
    bodies;

    /**
     * The solver algorithm to use. Default is GSSolver
     * @property solver
     * @type {Solver}
     */
    solver;

    /**
     * @property constraints
     * @type {Array}
     */
    constraints;

    /**
     * @property narrowphase
     * @type {Narrowphase}
     */
    narrowphase;

    /**
     * @property {ArrayCollisionMatrix} collisionMatrix
     * @type {ArrayCollisionMatrix}
     */
    collisionMatrix;

    /**
     * CollisionMatrix from the previous step.
     * @property {ArrayCollisionMatrix} collisionMatrixPrevious
     * @type {ArrayCollisionMatrix}
     */
    collisionMatrixPrevious;

    bodyOverlapKeeper;
    shapeOverlapKeeper;

    /**
     * All added materials
     * @property materials
     * @type {Array}
     */
    materials;

    /**
     * @property contactmaterials
     * @type {Array}
     */
    contactmaterials;

    /**
     * Used to look up a ContactMaterial given two instances of Material.
     * @property {TupleDictionary} contactMaterialTable
     */
    contactMaterialTable;

    defaultMaterial;

    /**
     * This contact material is used if no suitable contactmaterial is found for a contact.
     * @property defaultContactMaterial
     * @type {ContactMaterial}
     */
    defaultContactMaterial;

    /**
     * @property doProfiling
     * @type {Boolean}
     */
    doProfiling;

    /**
     * @property profile
     * @type {Object}
     */
    profile;

    /**
     * Time accumulator for interpolation. See http://gafferongames.com/game-physics/fix-your-timestep/
     * @property {Number} accumulator
     */
    accumulator = 0;

    /**
     * @property subsystems
     * @type {Array}
     */
    subsystems;

    /**
     * Dispatched after a body has been added to the world.
     * @event addBody
     * @param {Body} body The body that has been added to the world.
     */
    addBodyEvent;

    /**
     * Dispatched after a body has been removed from the world.
     * @event removeBody
     * @param {Body} body The body that has been removed from the world.
     */
    removeBodyEvent;

    idToBodyMap;

    /**
     * @param {object} [options]
     * @param {Vec3} [options.gravity]
     * @param {boolean} [options.allowSleep]
     * @param {Broadphase} [options.broadphase]
     * @param {Solver} [options.solver]
     * @param {boolean} [options.quatNormalizeFast]
     * @param {number} [options.quatNormalizeSkip]
     */
    constructor(options = {}) {
        super();

        const {
            allowSleep,
            quatNormalizeSkip = 0,
            quatNormalizeFast = false,
            gravity,
            broadphase = new NaiveBroadphase(),
            solver = new GSSolver()
        } = options;

        this.dt = -1;
        this.allowSleep = !!allowSleep;
        this.contacts = [];
        this.frictionEquations = [];
        this.quatNormalizeSkip = quatNormalizeSkip;
        this.quatNormalizeFast = quatNormalizeFast;
        this.time = 0.0;
        this.stepnumber = 0;
        /// Default and last timestep sizes
        this.default_dt = 1 / 60;
        this.nextId = 0;
        this.gravity = new Vec3();
        if (gravity) {
            this.gravity.copy(gravity);
        }

        this.broadphase = broadphase;
        this.bodies = [];
        this.solver = solver;
        this.constraints = [];
        this.narrowphase = new Narrowphase(this);
        this.collisionMatrix = new ArrayCollisionMatrix();
        this.collisionMatrixPrevious = new ArrayCollisionMatrix();
        this.bodyOverlapKeeper = new OverlapKeeper();
        this.shapeOverlapKeeper = new OverlapKeeper();
        this.materials = [];
        this.contactmaterials = [];
        this.contactMaterialTable = new TupleDictionary();
        this.defaultMaterial = new Material("default");
        this.defaultContactMaterial = new ContactMaterial(this.defaultMaterial, this.defaultMaterial, {
            friction: 0.3,
            restitution: 0.0
        });
        this.doProfiling = false;
        this.profile = {
            solve: 0,
            makeContactConstraints: 0,
            broadphase: 0,
            integrate: 0,
            narrowphase: 0,
        };
        this.accumulator = 0;
        this.subsystems = [];
        this.addBodyEvent = {
            type: "addBody",
            body: null
        };
        this.removeBodyEvent = {
            type: "removeBody",
            body: null
        };
        this.idToBodyMap = {};

        this.broadphase.setWorld(this);
    }

    /**
     * Get the contact material between materials m1 and m2
     * @method getContactMaterial
     * @param {Material} m1
     * @param {Material} m2
     * @return {ContactMaterial} The contact material if it was found.
     */
    getContactMaterial(m1, m2) {
        return this.contactMaterialTable.get(m1.id, m2.id); //this.contactmaterials[this.mats2cmat[i+j*this.materials.length]];
    }

    /**
     * Get number of objects in the world.
     * @method numObjects
     * @return {Number}
     * @deprecated
     */
    numObjects() {
        return this.bodies.length;
    }

    /**
     * Store old collision state info
     * @method collisionMatrixTick
     */
    collisionMatrixTick() {
        var temp = this.collisionMatrixPrevious;
        this.collisionMatrixPrevious = this.collisionMatrix;
        this.collisionMatrix = temp;
        this.collisionMatrix.reset();

        this.bodyOverlapKeeper.tick();
        this.shapeOverlapKeeper.tick();
    }

    /**
     * Add a rigid body to the simulation.
     * @method add
     * @param {Body} body
     * @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
     * @todo Adding an array of bodies should be possible. This would save some loops too
     */
    addBody(body) {
        if (this.bodies.indexOf(body) !== -1) {
            return;
        }
        body.index = this.bodies.length;
        this.bodies.push(body);
        body.world = this;
        body.initPosition.copy(body.position);
        body.initVelocity.copy(body.velocity);
        body.timeLastSleepy = this.time;
        if (body instanceof Body) {
            body.initAngularVelocity.copy(body.angularVelocity);
            body.initQuaternion.copy(body.quaternion);
        }
        this.collisionMatrix.setNumObjects(this.bodies.length);
        this.addBodyEvent.body = body;
        this.idToBodyMap[body.id] = body;
        this.dispatchEvent(this.addBodyEvent);
    }

    /**
     * Add a constraint to the simulation.
     * @method addConstraint
     * @param {Constraint} c
     */
    addConstraint(c) {
        this.constraints.push(c);
    }

    /**
     * Removes a constraint
     * @method removeConstraint
     * @param {Constraint} c
     */
    removeConstraint(c) {
        const idx = this.constraints.indexOf(c);
        if (idx !== -1) {
            this.constraints.splice(idx, 1);
        }
    }

    /**
     * Raycast test
     * @method rayTest
     * @param {Vec3} from
     * @param {Vec3} to
     * @param {RaycastResult} result
     * @deprecated Use .raycastAll, .raycastClosest or .raycastAny instead.
     */
    rayTest(from, to, result) {
        if (result instanceof RaycastResult) {
            // Do raycastclosest
            this.raycastClosest(from, to, {
                skipBackfaces: true
            }, result);
        } else {
            // Do raycastAll
            this.raycastAll(from, to, {
                skipBackfaces: true
            }, result);
        }
    }

    /**
     * Ray cast against all bodies. The provided callback will be executed for each hit with a RaycastResult as single argument.
     * @method raycastAll
     * @param  {Vec3} from
     * @param  {Vec3} to
     * @param  {Object} options
     * @param  {Function} callback
     * @return {boolean} True if any body was hit.
     */
    raycastAll(from, to, options, callback) {
        options.mode = Ray.ALL;
        options.from = from;
        options.to = to;
        options.callback = callback;
        return tmpRay.intersectWorld(this, options);
    }

    /**
     * Ray cast, and stop at the first result. Note that the order is random - but the method is fast.
     * @method raycastAny
     * @param  {Vec3} from
     * @param  {Vec3} to
     * @param  {Object} options
     * @param  {RaycastResult} result
     * @return {boolean} True if any body was hit.
     */
    raycastAny(from, to, options, result) {
        options.mode = Ray.ANY;
        options.from = from;
        options.to = to;
        options.result = result;
        return tmpRay.intersectWorld(this, options);
    }

    /**
     * Ray cast, and return information of the closest hit.
     * @method raycastClosest
     * @param  {Vec3} from
     * @param  {Vec3} to
     * @param  {Object} options
     * @param  {RaycastResult} result
     * @return {boolean} True if any body was hit.
     */
    raycastClosest(from, to, options, result) {
        options.mode = Ray.CLOSEST;
        options.from = from;
        options.to = to;
        options.result = result;
        return tmpRay.intersectWorld(this, options);
    }

    /**
     * Remove a rigid body from the simulation.
     * @method remove
     * @param {Body} body
     */
    removeBody(body) {
        body.world = null;
        const n = this.bodies.length - 1,
            bodies = this.bodies,
            idx = bodies.indexOf(body);

        if (idx !== -1) {
            bodies.splice(idx, 1); // Todo: should use a garbage free method

            // Recompute index
            for (let i = 0; i !== bodies.length; i++) {
                bodies[i].index = i;
            }

            this.collisionMatrix.setNumObjects(n);
            this.removeBodyEvent.body = body;
            delete this.idToBodyMap[body.id];
            this.dispatchEvent(this.removeBodyEvent);
        }
    }

    getBodyById(id) {
        return this.idToBodyMap[id];
    }

    // TODO Make a faster map
    getShapeById(id) {
        const bodies = this.bodies;
        for (let i = 0, bl = bodies.length; i < bl; i++) {
            const shapes = bodies[i].shapes;
            for (let j = 0, sl = shapes.length; j < sl; j++) {
                const shape = shapes[j];
                if (shape.id === id) {
                    return shape;
                }
            }
        }
    }

    /**
     * Adds a material to the World.
     * @method addMaterial
     * @param {Material} m
     * @todo Necessary?
     */
    addMaterial(m) {
        this.materials.push(m);
    }

    /**
     * Adds a contact material to the World
     * @method addContactMaterial
     * @param {ContactMaterial} cmat
     */
    addContactMaterial(cmat) {

        // Add contact material
        this.contactmaterials.push(cmat);

        // Add current contact material to the material table
        this.contactMaterialTable.set(cmat.materials[0].id, cmat.materials[1].id, cmat);
    }

    /**
     * Step the physics world forward in time.
     *
     * There are two modes. The simple mode is fixed timestepping without interpolation. In this case you only use the first argument. The second case uses interpolation. In that you also provide the time since the function was last used, as well as the maximum fixed timesteps to take.
     *
     * @method step
     * @param {Number} dt                       The fixed time step size to use.
     * @param {Number} [timeSinceLastCalled]    The time elapsed since the function was last called.
     * @param {Number} [maxSubSteps=10]         Maximum number of fixed steps to take per function call.
     *
     * @example
     *     // fixed timestepping without interpolation
     *     world.step(1/60);
     *
     * @see http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
     */
    step(dt, timeSinceLastCalled = 0, maxSubSteps = 10) {
        if (timeSinceLastCalled === 0) { // Fixed, simple stepping
            this.internalStep(dt);

            // Increment time
            this.time += dt;

        } else {
            this.accumulator += timeSinceLastCalled;
            let substeps = 0;
            while (this.accumulator >= dt && substeps < maxSubSteps) {
                // Do fixed steps to catch up
                this.internalStep(dt);
                this.accumulator -= dt;
                substeps++;
            }

            const t = (this.accumulator % dt) / dt;
            for (let j = 0; j !== this.bodies.length; j++) {
                const b = this.bodies[j];
                b.previousPosition.lerp(b.position, t, b.interpolatedPosition);
                b.previousQuaternion.slerp(b.quaternion, t, b.interpolatedQuaternion);
                b.previousQuaternion.normalize();
            }
            this.time += timeSinceLastCalled;
        }
    }

    internalStep(dt) {
        this.dt = dt;

        const contacts = this.contacts,
            p1 = World_step_p1,
            p2 = World_step_p2,
            N = this.numObjects(),
            bodies = this.bodies,
            solver = this.solver,
            gravity = this.gravity,
            doProfiling = this.doProfiling,
            profile = this.profile,
            DYNAMIC = Body.DYNAMIC,
            constraints = this.constraints,
            frictionEquationPool = World_step_frictionEquationPool,
            gx = gravity.x,
            gy = gravity.y,
            gz = gravity.z;


        let profilingStart;
        let i;

        if (doProfiling) {
            profilingStart = performance.now();
        }

        // Add gravity to all objects
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.type === DYNAMIC && bi.enableGravity) { // Only for dynamic bodies
                const f = bi.force,
                    m = bi.mass,
                    gf = bi.gravityFactor;

                f.x += m * gx * gf.x;
                f.y += m * gy * gf.y;
                f.z += m * gz * gf.z;
            }
        }

        // Update subsystems
        for (let i = 0, Nsubsystems = this.subsystems.length; i !== Nsubsystems; i++) {
            this.subsystems[i].update();
        }

        // Collision detection
        if (doProfiling) {
            profilingStart = performance.now();
        }
        p1.length = 0; // Clean up pair arrays from last step
        p2.length = 0;
        this.broadphase.collisionPairs(this, p1, p2);
        if (doProfiling) {
            profile.broadphase = performance.now() - profilingStart;
        }

        // Remove constrained pairs with collideConnected == false
        let Nconstraints = constraints.length;
        for (i = 0; i !== Nconstraints; i++) {
            const c = constraints[i];
            if (!c.collideConnected) {
                for (let j = p1.length - 1; j >= 0; j -= 1) {
                    if ((c.bodyA === p1[j] && c.bodyB === p2[j]) ||
                        (c.bodyB === p1[j] && c.bodyA === p2[j])) {
                        p1.splice(j, 1);
                        p2.splice(j, 1);
                    }
                }
            }
        }

        this.collisionMatrixTick();

        // Generate contacts
        if (doProfiling) {
            profilingStart = performance.now();
        }
        const oldcontacts = World_step_oldContacts;
        const NoldContacts = contacts.length;

        for (i = 0; i !== NoldContacts; i++) {
            oldcontacts.push(contacts[i]);
        }
        contacts.length = 0;

        // Transfer FrictionEquation from current list to the pool for reuse
        const NoldFrictionEquations = this.frictionEquations.length;
        for (i = 0; i !== NoldFrictionEquations; i++) {
            frictionEquationPool.push(this.frictionEquations[i]);
        }
        this.frictionEquations.length = 0;

        this.narrowphase.getContacts(
            p1,
            p2,
            this,
            contacts,
            oldcontacts, // To be reused
            this.frictionEquations,
            frictionEquationPool
        );

        if (doProfiling) {
            profile.narrowphase = performance.now() - profilingStart;
        }

        // Loop over all collisions
        if (doProfiling) {
            profilingStart = performance.now();
        }

        // Add all friction eqs
        for (i = 0; i < this.frictionEquations.length; i++) {
            solver.addEquation(this.frictionEquations[i]);
        }

        const ncontacts = contacts.length;
        for (let k = 0; k !== ncontacts; k++) {

            // Current contact
            const c = contacts[k];

            // Get current collision indeces
            const bi = c.bi,
                bj = c.bj,
                si = c.si,
                sj = c.sj;

            // Get collision properties
            let cm;
            if (bi.material && bj.material) {
                cm = this.getContactMaterial(bi.material, bj.material) || this.defaultContactMaterial;
            } else {
                cm = this.defaultContactMaterial;
            }

            // c.enabled = bi.collisionResponse && bj.collisionResponse && si.collisionResponse && sj.collisionResponse;

            let mu = cm.friction;
            // c.restitution = cm.restitution;

            // If friction or restitution were specified in the material, use them
            if (bi.material && bj.material) {
                if (bi.material.friction >= 0 && bj.material.friction >= 0) {
                    mu = bi.material.friction * bj.material.friction;
                }

                if (bi.material.restitution >= 0 && bj.material.restitution >= 0) {
                    c.restitution = bi.material.restitution * bj.material.restitution;
                }
            }

            // c.setSpookParams(
            //           cm.contactEquationStiffness,
            //           cm.contactEquationRelaxation,
            //           dt
            //       );

            solver.addEquation(c);

            // // Add friction constraint equation
            // if(mu > 0){

            // 	// Create 2 tangent equations
            // 	var mug = mu * gnorm;
            // 	var reducedMass = (bi.invMass + bj.invMass);
            // 	if(reducedMass > 0){
            // 		reducedMass = 1/reducedMass;
            // 	}
            // 	var pool = frictionEquationPool;
            // 	var c1 = pool.length ? pool.pop() : new FrictionEquation(bi,bj,mug*reducedMass);
            // 	var c2 = pool.length ? pool.pop() : new FrictionEquation(bi,bj,mug*reducedMass);
            // 	this.frictionEquations.push(c1, c2);

            // 	c1.bi = c2.bi = bi;
            // 	c1.bj = c2.bj = bj;
            // 	c1.minForce = c2.minForce = -mug*reducedMass;
            // 	c1.maxForce = c2.maxForce = mug*reducedMass;

            // 	// Copy over the relative vectors
            // 	c1.ri.copy(c.ri);
            // 	c1.rj.copy(c.rj);
            // 	c2.ri.copy(c.ri);
            // 	c2.rj.copy(c.rj);

            // 	// Construct tangents
            // 	c.ni.tangents(c1.t, c2.t);

            //           // Set spook params
            //           c1.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, dt);
            //           c2.setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, dt);

            //           c1.enabled = c2.enabled = c.enabled;

            // 	// Add equations to solver
            // 	solver.addEquation(c1);
            // 	solver.addEquation(c2);
            // }

            if (bi.allowSleep &&
                bi.type === Body.DYNAMIC &&
                bi.sleepState === Body.SLEEPING &&
                bj.sleepState === Body.AWAKE &&
                bj.type !== Body.STATIC
            ) {
                const speedSquaredB = bj.velocity.norm2() + bj.angularVelocity.norm2();
                const speedLimitSquaredB = Math.pow(bj.sleepSpeedLimit, 2);
                if (speedSquaredB >= speedLimitSquaredB * 2) {
                    bi._wakeUpAfterNarrowphase = true;
                }
            }

            if (bj.allowSleep &&
                bj.type === Body.DYNAMIC &&
                bj.sleepState === Body.SLEEPING &&
                bi.sleepState === Body.AWAKE &&
                bi.type !== Body.STATIC
            ) {
                const speedSquaredA = bi.velocity.norm2() + bi.angularVelocity.norm2();
                const speedLimitSquaredA = Math.pow(bi.sleepSpeedLimit, 2);
                if (speedSquaredA >= speedLimitSquaredA * 2) {
                    bj._wakeUpAfterNarrowphase = true;
                }
            }

            // Now we know that i and j are in contact. Set collision matrix state
            this.collisionMatrix.set(bi, bj, true);

            if (!this.collisionMatrixPrevious.get(bi, bj)) {
                // First contact!
                // We reuse the collideEvent object, otherwise we will end up creating new objects for each new contact, even if there's no event listener attached.
                World_step_collideEvent.body = bj;
                World_step_collideEvent.contact = c;
                bi.dispatchEvent(World_step_collideEvent);

                World_step_collideEvent.body = bi;
                bj.dispatchEvent(World_step_collideEvent);
            }

            this.bodyOverlapKeeper.set(bi.id, bj.id);
            this.shapeOverlapKeeper.set(si.id, sj.id);
        }

        this.emitContactEvents();

        if (doProfiling) {
            profile.makeContactConstraints = performance.now() - profilingStart;
            profilingStart = performance.now();
        }

        // Wake up bodies
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi._wakeUpAfterNarrowphase) {
                bi.wakeUp();
                bi._wakeUpAfterNarrowphase = false;
            }
        }

        // Add user-added constraints
        Nconstraints = constraints.length;
        for (i = 0; i !== Nconstraints; i++) {
            const c = constraints[i];
            c.update();
            for (let j = 0, Neq = c.equations.length; j !== Neq; j++) {
                const eq = c.equations[j];
                solver.addEquation(eq);
            }
        }

        // Solve the constrained system
        solver.solve(dt, this);

        if (doProfiling) {
            profile.solve = performance.now() - profilingStart;
        }

        // Remove all contacts from solver
        solver.removeAllEquations();

        // Apply damping, see http://code.google.com/p/bullet/issues/detail?id=74 for details
        const pow = Math.pow;
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.type & DYNAMIC) { // Only for dynamic bodies
                const ld = pow(1.0 - bi.linearDamping, dt);
                const v = bi.velocity;
                v.mult(ld, v);
                const av = bi.angularVelocity;
                if (av) {
                    const ad = pow(1.0 - bi.angularDamping, dt);
                    av.mult(ad, av);
                }
            }
        }

        this.dispatchEvent(World_step_preStepEvent);

        // Invoke pre-step callbacks
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.preStep) {
                bi.preStep.call(bi);
            }
        }

        // Leap frog
        // vnew = v + h*f/m
        // xnew = x + h*vnew
        if (doProfiling) {
            profilingStart = performance.now();
        }
        const stepnumber = this.stepnumber;
        const quatNormalize = stepnumber % (this.quatNormalizeSkip + 1) === 0;
        const quatNormalizeFast = this.quatNormalizeFast;

        for (i = 0; i !== N; i++) {
            bodies[i].integrate(dt, quatNormalize, quatNormalizeFast);
        }
        this.clearForces();

        this.broadphase.dirty = true;

        if (doProfiling) {
            profile.integrate = performance.now() - profilingStart;
        }

        // Update world time
        this.time += dt;
        this.stepnumber += 1;

        this.dispatchEvent(World_step_postStepEvent);

        // Invoke post-step callbacks
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            const postStep = bi.postStep;
            if (postStep) {
                postStep.call(bi);
            }
        }

        // Sleeping update
        if (this.allowSleep) {
            for (i = 0; i !== N; i++) {
                bodies[i].sleepTick(this.time);
            }
        }
    }

    emitContactEvents() {
        const hasBeginContact = this.hasAnyEventListener('beginContact');
        const hasEndContact = this.hasAnyEventListener('endContact');

        if (hasBeginContact || hasEndContact) {
            this.bodyOverlapKeeper.getDiff(additions, removals);
        }

        if (hasBeginContact) {
            for (let i = 0, l = additions.length; i < l; i += 2) {
                beginContactEvent.bodyA = this.getBodyById(additions[i]);
                beginContactEvent.bodyB = this.getBodyById(additions[i + 1]);
                this.dispatchEvent(beginContactEvent);
            }
            beginContactEvent.bodyA = beginContactEvent.bodyB = null;
        }

        if (hasEndContact) {
            for (let i = 0, l = removals.length; i < l; i += 2) {
                endContactEvent.bodyA = this.getBodyById(removals[i]);
                endContactEvent.bodyB = this.getBodyById(removals[i + 1]);
                this.dispatchEvent(endContactEvent);
            }
            endContactEvent.bodyA = endContactEvent.bodyB = null;
        }

        additions.length = removals.length = 0;

        const hasBeginShapeContact = this.hasAnyEventListener('beginShapeContact');
        const hasEndShapeContact = this.hasAnyEventListener('endShapeContact');

        if (hasBeginShapeContact || hasEndShapeContact) {
            this.shapeOverlapKeeper.getDiff(additions, removals);
        }

        if (hasBeginShapeContact) {
            for (let i = 0, l = additions.length; i < l; i += 2) {
                const shapeA = this.getShapeById(additions[i]);
                const shapeB = this.getShapeById(additions[i + 1]);
                beginShapeContactEvent.shapeA = shapeA;
                beginShapeContactEvent.shapeB = shapeB;
                beginShapeContactEvent.bodyA = shapeA.body;
                beginShapeContactEvent.bodyB = shapeB.body;
                this.dispatchEvent(beginShapeContactEvent);
            }
            beginShapeContactEvent.bodyA = beginShapeContactEvent.bodyB = beginShapeContactEvent.shapeA = beginShapeContactEvent.shapeB = null;
        }

        if (hasEndShapeContact) {
            for (let i = 0, l = removals.length; i < l; i += 2) {
                const shapeA = this.getShapeById(removals[i]);
                const shapeB = this.getShapeById(removals[i + 1]);
                endShapeContactEvent.shapeA = shapeA;
                endShapeContactEvent.shapeB = shapeB;
                endShapeContactEvent.bodyA = shapeA.body;
                endShapeContactEvent.bodyB = shapeB.body;
                this.dispatchEvent(endShapeContactEvent);
            }
            endShapeContactEvent.bodyA = endShapeContactEvent.bodyB = endShapeContactEvent.shapeA = endShapeContactEvent.shapeB = null;
        }
    }

    /**
     * Sets all body forces in the world to zero.
     * @method clearForces
     */
    clearForces() {
        const bodies = this.bodies;
        const N = bodies.length;
        for (let i = 0; i !== N; i++) {
            const b = bodies[i];
            const force = b.force;
            const tau = b.torque;

            force.set(0, 0, 0);
            tau.set(0, 0, 0);
        }
    }
}

World.prototype.add = World.prototype.addBody;
World.prototype.remove = World.prototype.removeBody;