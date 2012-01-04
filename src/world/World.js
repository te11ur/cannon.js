/**
 * The physics world
 * @class World
 */
PHYSICS.World = function(){

  // Some default values
  this.paused = false;
  this.time = 0.0;
  this.stepnumber = 0;
  this.iter = 10;

  this.spook_k = 300.0;
  this.spook_d = 1;

  var th = this;
  this.spook_a = function(h){ return 4.0 / (h * (1 + 4 * th.spook_d)); };
  this.spook_b = (4.0 * this.spook_d) / (1 + 4 * this.spook_d);
  this.spook_eps = function(h){ return 4.0 / (h * h * th.spook_k * (1 + 4 * th.spook_d)); };
};

/**
 * Get number of objects in the world.
 * @return int
 */
PHYSICS.World.prototype.togglepause = function(){
  this.paused = !this.paused;
};

/**
 * Get number of objects in the world.
 * @return int
 */
PHYSICS.World.prototype.numObjects = function(){
  return this.x ? this.x.length : 0;
};

/**
 * Add a rigid body to the simulation.
 * @param RigidBody body
 * @todo If the simulation has not yet started, why recrete and copy arrays for each body? Accumulate in dynamic arrays in this case.
 * @todo Adding an array of bodies should be possible. This would save some loops too
 */
PHYSICS.World.prototype.add = function(body){
  if(!body)
    return;

  var n = this.numObjects();

  old_x = this.x;
  old_y = this.y;
  old_z = this.z;
  
  old_vx = this.vx;
  old_vy = this.vy;
  old_vz = this.vz;
  
  old_fx = this.fx;
  old_fy = this.fy;
  old_fz = this.fz;
  
  old_taux = this.taux;
  old_tauy = this.tauy;
  old_tauz = this.tauz;
  
  old_wx = this.wx;
  old_wy = this.wy;
  old_wz = this.wz;
  
  old_qx = this.qx;
  old_qy = this.qy;
  old_qz = this.qz;
  old_qw = this.qw;

  old_type = this.type;
  old_geodata = this.geodata;
  old_body = this.body;
  old_fixed = this.fixed;
  old_invm = this.invm;
  old_mass = this.mass;
  old_inertiax = this.inertiax;
  old_inertiay = this.inertiay;
  old_inertiaz = this.inertiaz;

  this.x = new Float32Array(n+1);
  this.y = new Float32Array(n+1);
  this.z = new Float32Array(n+1);
  
  this.vx = new Float32Array(n+1);
  this.vy = new Float32Array(n+1);
  this.vz = new Float32Array(n+1);
  
  this.fx = new Float32Array(n+1);
  this.fy = new Float32Array(n+1);
  this.fz = new Float32Array(n+1);
  
  this.taux = new Float32Array(n+1);
  this.tauy = new Float32Array(n+1);
  this.tauz = new Float32Array(n+1);
  
  this.wx = new Float32Array(n+1);
  this.wy = new Float32Array(n+1);
  this.wz = new Float32Array(n+1);
  
  this.qx = new Float32Array(n+1);
  this.qy = new Float32Array(n+1);
  this.qz = new Float32Array(n+1);
  this.qw = new Float32Array(n+1);

  this.type = new Int16Array(n+1);
  this.geodata = [];
  this.body = [];
  this.fixed = new Int16Array(n+1);
  this.mass = new Float32Array(n+1);
  this.inertiax = new Float32Array(n+1);
  this.inertiay = new Float32Array(n+1);
  this.inertiaz = new Float32Array(n+1);
  this.invm = new Float32Array(n+1);
  
  // Add old data to new array
  for(var i=0; i<n; i++){
    this.x[i] = old_x[i];
    this.y[i] = old_y[i];
    this.z[i] = old_z[i];
  
    this.vx[i] = old_vx[i];
    this.vy[i] = old_vy[i];
    this.vz[i] = old_vz[i];
  
    this.fx[i] = old_fx[i];
    this.fy[i] = old_fy[i];
    this.fz[i] = old_fz[i];
  
    this.taux[i] = old_taux[i];
    this.tauy[i] = old_tauy[i];
    this.tauz[i] = old_tauz[i];
  
    this.wx[i] = old_wx[i];
    this.wy[i] = old_wy[i];
    this.wz[i] = old_wz[i];
  
    this.qx[i] = old_qx[i];
    this.qy[i] = old_qy[i];
    this.qz[i] = old_qz[i];
    this.qw[i] = old_qw[i];

    this.type[i] = old_type[i];
    this.geodata[i] = old_geodata[i];
    this.body[i] = old_body[i];
    this.fixed[i] = old_geodata[i];
    this.invm[i] = old_invm[i];
    this.mass[i] = old_mass[i];
    this.inertiax[i] = old_inertiax[i];
    this.inertiay[i] = old_inertiay[i];
    this.inertiaz[i] = old_inertiaz[i];
  }

  // Add one more
  this.x[n] = body.position.x;
  this.y[n] = body.position.y;
  this.z[n] = body.position.z;
  
  this.vx[n] = body.velocity.x;
  this.vy[n] = body.velocity.y;
  this.vz[n] = body.velocity.z;
  
  this.fx[n] = body.force.x;
  this.fy[n] = body.force.y;
  this.fz[n] = body.force.z;
  
  this.taux[n] = body.tau.x;
  this.tauy[n] = body.tau.y;
  this.tauz[n] = body.tau.z;

  this.wx[n] = body.rotvelo.x;
  this.wy[n] = body.rotvelo.y;
  this.wz[n] = body.rotvelo.z;
  
  this.qx[n] = body.quaternion.x;
  this.qy[n] = body.quaternion.y;
  this.qz[n] = body.quaternion.z;
  this.qw[n] = body.quaternion.w;

  this.type[n] = body.type;
  this.geodata[n] = body.geodata;
  this.body[n] = body; // Keep reference to body
  this.fixed[n] = body.mass<=0.0;
  this.invm[n] = body.mass>0 ? 1.0/body.mass : 0;
  this.mass[n] = body.mass;

  this.inertiax[n] = body.inertia.x;
  this.inertiay[n] = body.inertia.y;
  this.inertiaz[n] = body.inertia.z;

  body.id = n-1; // give id as index in table
  body.world = this;

  // Create collision matrix
  this.collision_matrix = new Int16Array((n+1)*(n+1));
};

/**
 * Get/set the broadphase collision detector for the world.
 * @param BroadPhase broadphase
 * @return BroadPhase
 */
PHYSICS.World.prototype.broadphase = function(broadphase){
  if(broadphase){
    this._broadphase = broadphase;
  } else
    return this._broadphase;
};

/**
 * Get/set the number of iterations
 * @param int n
 * @return int
 */
PHYSICS.World.prototype.iterations = function(n){
  if(n)
    this.iter = parseInt(n);
  else
    return this.iter;
};

/**
 * Set the gravity
 * @param Vec3
 * @return Vec3
 */
PHYSICS.World.prototype.gravity = function(g){
  if(g==undefined)
    return this.gravity;
  else
    this.gravity = g;
};

/**
 * Step the simulation
 * @param float dt
 */
PHYSICS.World.prototype.step = function(dt){
  if(this.paused)
    return;

  // 1. Collision detection
  var pairs = this._broadphase.collisionPairs(this);
  var p1 = pairs[0];
  var p2 = pairs[1];

  // Get references to things that are accessed often. Will save some lookup time.
  var SPHERE = PHYSICS.RigidBody.prototype.types.SPHERE;
  var PLANE = PHYSICS.RigidBody.prototype.types.PLANE;
  var types = world.type;
  var x = world.x;
  var y = world.y;
  var z = world.z;
  var qx = world.qx;
  var qy = world.qy;
  var qz = world.qz;
  var qw = world.qw;
  var vx = world.vx;
  var vy = world.vy;
  var vz = world.vz;
  var wx = world.wx;
  var wy = world.wy;
  var wz = world.wz;
  var fx = world.fx;
  var fy = world.fy;
  var fz = world.fz;
  var taux = world.taux;
  var tauy = world.tauy;
  var tauz = world.tauz;

  // @todo reuse these somehow?
  var vx_lambda = new Float32Array(world.x.length);
  var vy_lambda = new Float32Array(world.y.length);
  var vz_lambda = new Float32Array(world.z.length);
  var wx_lambda = new Float32Array(world.x.length);
  var wy_lambda = new Float32Array(world.y.length);
  var wz_lambda = new Float32Array(world.z.length);

  var lambdas = new Float32Array(p1.length);
  for(var i=0; i<lambdas.length; i++){
    lambdas[i] = 0;
    vx_lambda[i] = 0;
    vy_lambda[i] = 0;
    vz_lambda[i] = 0;
    wx_lambda[i] = 0;
    wy_lambda[i] = 0;
    wz_lambda[i] = 0;
  }

  var that = this;
  function cmatrix(i,j,newval){
    if(i>j){
      var temp = j;
      j = i;
      i = temp;
    }
    if(newval===undefined)
      return that.collision_matrix[i+j*that.numObjects()];
    else {
      that.collision_matrix[i+j*that.numObjects()] = parseInt(newval);
    }
  }

  // Resolve impulses
  for(var k=0; k<p1.length; k++){

    // Get current collision indeces
    var i = p1[k];
    var j = p2[k];
      
    // sphere-plane collision
    if((types[i]==SPHERE &&
	types[j]==PLANE) ||
       (types[i]==PLANE &&
	types[j]==SPHERE)){
	
      // Identify what is what
      var pi, si;
      if(types[i]==SPHERE){
	si=i;
	pi=j;
      } else {
	si=j;
	pi=i;
      }
      
      // Collision normal
      var n = world.geodata[pi].normal;
	
      // Check if penetration
      var r = new PHYSICS.Vec3(x[si]-x[pi],
			       y[si]-y[pi],
			       z[si]-z[pi]);
      r = n.mult(r.dot(n));
      var q = (r.dot(n)-world.geodata[si].radius);

      var w_sphere = new PHYSICS.Vec3(wx[si], wy[si], wz[si]);
      var v_sphere = new PHYSICS.Vec3(vx[si], vy[si], vz[si]);
      // Contact velocity
      // v = (body(n).V(1:3) + cr(body(n).V(4:6)',rn)') - (body(m).V(1:3) + cr(body(m).V(4:6)',rm)');
      // @todo

      var cr = v_sphere.vadd(v_sphere.cross(r)); // OK

      var v_contact = new PHYSICS.Vec3(vx[si]+cr.x,
				       vy[si]+cr.y,
				       vz[si]+cr.z); // OK
     
      v_sphere.vadd(w_sphere.cross(r),v_contact);

      // Relative velocity
      var u = n.mult(v_sphere.dot(n));
      
      // Action if penetration
      if(q<=0.0 && cmatrix(si,pi)==0){ // No impact for separating contacts
	if(u.dot(n)<0.0)
	  cmatrix(si,pi,1);
	var r_star = r.crossmat();
	
	var invm = this.invm;
	// Collision matrix:
	// K = eye(3,3)/body(n).m - r_star*body(n).Iinv*r_star;
	var K = new PHYSICS.Mat3();
	K.identity();
	K.elements[0] *= invm[si];
	K.elements[4] *= invm[si];
	K.elements[8] *= invm[si];

	var rIr = r_star.mmult(K.mmult(r_star));
	for(var el = 0; el<9; el++)
	  K.elements[el] -= rIr.elements[el];
	
	// First assume stick friction
	var e = 0.8;

	// Final velocity if stick
	var v_f = n.mult((1-e) * (v_contact.dot(n))); 

	var impulse_vec =  K.solve(v_f.vsub(v_contact));
	
	// Check if slide mode (J_t > J_n) - outside friction cone
	var mu = 0.3; // quick fix
	if(mu>0){
	  var J_n = n.mult(impulse_vec.dot(n));
	  var J_t = impulse_vec.vsub(J_n);
	  if(J_t.norm() > J_n.mult(mu).norm()){
	    var v_tang = v_sphere.vsub(n.mult(v_sphere.dot(n)));
	    var tangent = v_tang.mult(1/(v_tang.norm() + 0.0001));


	    var impulse = -(1+e)*(v_sphere.dot(n))/(n.dot(K.vmult((n.vsub(tangent.mult(mu))))));
	    impulse_vec = n.mult(impulse).vsub(tangent.mult(mu * impulse));
	  }
	}

	// Add to velocity
	// todo: add to angular velocity as below
	var add = impulse_vec.mult(invm[si]);
	vx[si] += add.x;
	vy[si] += add.y;
	vz[si] += add.z;

	var cr = impulse_vec.cross(r);
	var wadd = cr.mult(world.mass[si]);
	wx[si] += wadd.x; //body(n).V(4:6) = body(n).V(4:6) + (body(n).Iinv*cr(impulse_vec,r))';
	wy[si] += wadd.y;
	wz[si] += wadd.z;
	
	cmatrix(si,pi,-1); // Just applied impulse - set impact
      } else if(q<=0 & cmatrix(si,pi)==-1)
	cmatrix(si,pi,1); // Last step was impact and we are still penetrated- set contact
      else if(q>0)
	cmatrix(si,pi,0); // No penetration any more- set no contact

    } else if(types[i]==SPHERE &&
	      types[j]==SPHERE){

      var n = new PHYSICS.Vec3(x[i]-x[j],
			       y[i]-y[j],
			       z[i]-z[j]);
      n.normalize();
      var q = (nlen - (world.geodata[i].radius+world.geodata[j].radius));
      var u = new PHYSICS.Vec3(vx[i]-vx[j],
			       vy[i]-vy[j],
			       vz[i]-vz[j]);
      u = n.mult(u.dot(n));
      if(q<0.0 && u.dot(n)<0){
	var e = 0.5;
	var u_new = n.mult(-(u.dot(n)*e));
	
	vx[i] += e*(u_new.x - u.x)*invm[j];
	vy[i] += e*(u_new.y - u.y)*invm[j];
	vz[i] += e*(u_new.z - u.z)*invm[j];

	vx[j] -= e*(u_new.x - u.x)*invm[i];
	vy[j] -= e*(u_new.y - u.y)*invm[i];
	vz[j] -= e*(u_new.z - u.z)*invm[i];

	// Todo, implement below things. They are general impulses from granular.m
	var r = new PHYSICS.Vec3(x[i]-x[j],
				 y[i]-y[j],
				 z[i]-z[j]);
	var ri = n.mult(world.geodata[i].radius);
	var rj = n.mult(world.geodata[j].radius);

	/*
            % Collide with core
                r = dR;
                rn = -body(n).r_core * normal;
                rm = body(m).r_core * normal;
                v = (body(n).V(1:3) + cr(body(n).V(4:6)',rn)') - (body(m).V(1:3) + cr(body(m).V(4:6)',rm)');
                if v*r > 0 
                    COLLISION_MATRIX(n,m) = 1;
                    break                                                  % No impact for separating contacts
                end
                r_star = getSTAR2(r);
                rn_star = getSTAR2(rn);
                rm_star = getSTAR2(rm);
	*/

	var r_star = r.crossmat();
	var rn_star = rn.crossmat();
	var rm_star = rm.crossmat();

	/*

                K = eye(3,3)/body(n).m + eye(3,3)/body(m).m - rn_star*body(m).Iinv*rn_star - rm_star*body(n).Iinv*rm_star; 
                % First assume stick friction
                v_f = - e_pair * (v*normal) * normal';               % Final velocity if stick
                impulse_vec =  K\(v_f - v)';
                % Check if slide mode (J_t > J_n) - outside friction cone
                if MU>0
                    J_n = (impulse_vec'*normal) * normal;
                    J_t = impulse_vec - J_n;
                    if norm(J_t) > norm(MU*J_n)                    
                            v_tang = v' - (v*normal)*normal;
                            tangent =  v_tang/(norm(v_tang) + 10^(-6));
                            impulse = -(1+e_pair)*(v*normal)/(normal' * K * (normal - MU*tangent));
                            impulse_vec = impulse * normal - MU * impulse * tangent;
                    end
                end
                 bodyTotmass = body(n).m + body(m).m;
                 body(n).V(1:3) = body(n).V(1:3) +  1/body(n).m * impulse_vec';
                 body(n).V(4:6) = body(n).V(4:6) + (body(n).Iinv*cr(impulse_vec,rn))';
                 %body(n).x(1:3) = body(n).x(1:3) + penetration*normal * (body(n).m/bodyTotmass);
                 body(n).L = body(n).I*body(n).V(4:6)';
                 body(m).V(1:3) = body(m).V(1:3) -  1/body(m).m * impulse_vec';
                 body(m).V(4:6) = body(m).V(4:6) + (body(m).Iinv*cr(impulse_vec,rm))';
                 %body(m).x(1:3) = body(m).x(1:3) - penetration*normal * (body(m).m/bodyTotmass);
                 body(m).L = body(m).I*body(m).V(4:6)';
	 */


      }
    }
  }

  // Iterate over contacts
  for(var l=0; l<this.iterations(); l++){
    for(var k=0; k<p1.length; k++){

      // Get current collision indeces
      var i = p1[k];
      var j = p2[k];
      
      // sphere-plane collision
      if((types[i]==SPHERE &&
	  types[j]==PLANE) ||
	 (types[i]==PLANE &&
	  types[j]==SPHERE)){
	
	// Identify what is what
	var pi, si;
	if(types[i]==SPHERE){
	  si=i;
	  pi=j;
	} else {
	  si=j;
	  pi=i;
	}
	
	// Collision normal
	var n = world.geodata[pi].normal;
	
	// Check if penetration
	var r = new PHYSICS.Vec3(x[si]-x[pi],
				 y[si]-y[pi],
				 z[si]-z[pi]);
	var q = (r.dot(n)-world.geodata[si].radius)*2;
	var v_sphere = new PHYSICS.Vec3(vx[si],
					vy[si],
					vz[si]);
	
	var u = n.mult(v_sphere.dot(n));
	
	// Action if penetration
	if(q<0.0){

	  // Solve for lambda
	  var old_lambda = lambdas[k];
	  var fs = new PHYSICS.Vec3(fx[si],
				    fy[si],
				    fz[si]);
	  var new_deltalambda = (- q*world.spook_a(dt)
				 - u.dot(n)*world.spook_b
				 - (fs.dot(n)*world.invm[si])*dt
				 - old_lambda*world.spook_eps(dt))/(world.invm[si]
								    + 1/(world.mass[si]*Math.pow(world.geodata[si].radius,2.0/5.0))
								    + world.spook_eps(dt));
	  
	  var new_lambda = new_deltalambda - old_lambda; // + ?
	
	  // Check sign of lambdas and fix
	  if(new_lambda<0){
	    new_deltalambda = -new_lambda;
	    new_lambda = 0;
	  }
	  
	  // save for next timestep
	  lambdas[k] = new_lambda;
	  
	  // Accumulate velocities
	  vx_lambda[si] += n.x * new_deltalambda * world.invm[si];
	  vy_lambda[si] += n.y * new_deltalambda * world.invm[si];
	  vz_lambda[si] += n.z * new_deltalambda * world.invm[si];
	}
      } else if(types[i]==SPHERE &&
		types[j]==SPHERE){
	var r = new PHYSICS.Vec3(x[i]-x[j],
				 y[i]-y[j],
				 z[i]-z[j]);
	var nlen = r.norm();
	var n = new PHYSICS.Vec3(x[i]-x[j],
				 y[i]-y[j],
				 z[i]-z[j]);
	n.normalize();
	var q = (nlen - (world.geodata[i].radius+world.geodata[j].radius))*2;
	var u = new PHYSICS.Vec3(vx[i]-vx[j],
				 vy[i]-vy[j],
				 vz[i]-vz[j]);
	u = n.mult(u.dot(n));
	if(q<0.0){

	  // Solve for lambda
	  var old_lambda = lambdas[k];
	  var fi = new PHYSICS.Vec3(fx[i],
				    fy[i],
				    fz[i]);
	  var fj = new PHYSICS.Vec3(fx[j],
				    fy[j],
				    fz[j]);
	  var new_deltalambda = (- q*world.spook_a(dt)
				 - u.dot(n)*world.spook_b
				 - (fi.dot(n)*world.invm[i] + fj.dot(n)*world.invm[j])*dt
				 - old_lambda*world.spook_eps(dt))/(world.invm[i]
								    + world.invm[j]
								    + world.spook_eps(dt));
	
	  var new_lambda = new_deltalambda - old_lambda;
	
	  // Check sign of lambdas and fix
	  if(new_lambda < 0.0){
	    new_deltalambda = - new_lambda;
	    new_lambda = 0;
	  }
	
	  // save for next timestep
	  lambdas[k] = new_lambda;
	
	  // Accumulate velocities
	  vx_lambda[i] += n.x * new_deltalambda * world.invm[i];
	  vy_lambda[i] += n.y * new_deltalambda * world.invm[i];
	  vz_lambda[i] += n.z * new_deltalambda * world.invm[i];
	  vx_lambda[j] -= n.x * new_deltalambda * world.invm[j];
	  vy_lambda[j] -= n.y * new_deltalambda * world.invm[j];
	  vz_lambda[j] -= n.z * new_deltalambda * world.invm[j];

	  // Accumulate rotational velocities
	  // I.inv() is just the mass for spheres
	  // w_lambda[ij] = w_lambda[ij] +- I[ij].inv() * dlambda * (r x n)
	  var rxn = r.cross(n);
	  var Iinvi = world.mass[i];
	  var Iinvj = world.mass[j];
	  
	  wx_lambda[i] += Iinvi * new_deltalambda * rxn.x;
	  wy_lambda[i] += Iinvi * new_deltalambda * rxn.y;
	  wz_lambda[i] += Iinvi * new_deltalambda * rxn.z;
	  wx_lambda[j] -= Iinvj * new_deltalambda * rxn.x;
	  wy_lambda[j] -= Iinvj * new_deltalambda * rxn.y;
	  wz_lambda[j] -= Iinvj * new_deltalambda * rxn.z;
	}
      }
    }
  }

  // Add gravity to all objects
  for(var i=0; i<world.numObjects(); i++){
    if(!world.fixed[i]){
      fx[i] += world.gravity.x * world.mass[i];
      fy[i] += world.gravity.y * world.mass[i];
      fz[i] += world.gravity.z * world.mass[i];
    }
  }

  // Leap frog
  // vnew = v + h*f/m
  // xnew = x + h*vnew
  for(var i=0; i<world.numObjects(); i++){
    vx[i] += fx[i] * world.invm[i] * dt + vx_lambda[i];
    vy[i] += fy[i] * world.invm[i] * dt + vy_lambda[i];
    vz[i] += fz[i] * world.invm[i] * dt + vz_lambda[i];

    wx[i] += taux[i] * 1.0/world.inertiax[i] * dt + wx_lambda[i];
    wy[i] += tauy[i] * 1.0/world.inertiay[i] * dt + wy_lambda[i];
    wz[i] += tauz[i] * 1.0/world.inertiaz[i] * dt + wz_lambda[i];

    // Use new velocity  - leap frog
    x[i] += vx[i] * dt;
    y[i] += vy[i] * dt;
    z[i] += vz[i] * dt;
    
    var q = new PHYSICS.Quaternion(qx[i],qy[i],qz[i],qw[i]);
    var w = new PHYSICS.Quaternion(wx[i],wy[i],wz[i],0);
    var wq = w.mult(q);
    wq.normalize();
    qx[i] += dt * 0.5 * wq.x;
    qy[i] += dt * 0.5 * wq.y;
    qz[i] += dt * 0.5 * wq.z;
    qw[i] += dt * 0.5 * wq.w;
    
    q.x = qx[i];
    q.y = qy[i];
    q.z = qz[i];
    q.w = qw[i];
    q.normalize();
    qx[i]=q.x;
    qy[i]=q.y;
    qz[i]=q.z;
    qw[i]=q.w;
    
  }

  // Reset all forces
  for(var i = 0; i<world.numObjects(); i++){
    fx[i] = 0.0;
    fy[i] = 0.0;
    fz[i] = 0.0;
    taux[i] = 0.0;
    tauy[i] = 0.0;
    tauz[i] = 0.0;
  }

  // Update world time
  world.time += dt;
  world.stepnumber += 1;

  // Read all data into object references again
  for(var i=0; i<world.numObjects(); i++){
    world.body[i].position.x = x[i];
    world.body[i].position.y = y[i];
    world.body[i].position.z = z[i];

    world.body[i].rotvelo.x = vx[i];
    world.body[i].rotvelo.y = vy[i];
    world.body[i].rotvelo.z = vz[i];

    world.body[i].quaternion.x = qx[i];
    world.body[i].quaternion.y = qy[i];
    world.body[i].quaternion.z = qz[i];
    world.body[i].quaternion.w = qw[i];
  }  
};
