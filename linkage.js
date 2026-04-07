
class Linkage {
    constructor(origin, lengths, startAngles) {
      this._lengths = lengths;
      this._base = this._lengths[0]/1.5;
      this._origin = origin;
      this._solution = startAngles;
      this._prevSolution = [...startAngles];
      this._target = createVector(0, 0);
      this._vel = Array(this._lengths.length).fill(0);
      this._pos = [...startAngles];
    }
  
    f(angles) {  // Find end position using forward kinematics
      let x = this._origin.x - 0.5 * this._base;
      let y = this._origin.y;
      for (let i=0; i < this._lengths.length; i++) {
        x += -1 * this._lengths[i] * sin(angles[i]);
        y += this._lengths[i] * cos(angles[i]);
      }
      return createVector(x, y);
    }
  
    G(angles) {  // Find current error
      let f_k = this.f(angles);
      let penalty_fac = 0.9;
      let error = penalty_fac * euc_dist(angles, this._prevSolution);
      return this._target.dist(f_k) + error;
    }
  
    grad(angles, step_size) {  // approx. gradient using center diff
      let grad = [];
      for (let i=0; i < angles.length ; i++) {
        let a_pos = [...angles];
        let a_neg = [...angles];
        a_pos[i] += step_size;
        a_neg[i] -= step_size;
        grad.push((this.G(a_pos) - this.G(a_neg)) / (2 * step_size));
  
      }
      return grad;
    }
  
    gradDescent() {
      let phi;
      let phi_prev = [...this._prevSolution];
      let a = 0.007;
      let iterLimit = 1000;
      let step_size = 0.0001;
      let gradLimit = 0.01;
      let rot_min = 0.5;
      let rot_max = 2.6;
  
      for (let k=0; k < iterLimit; k++) {
        phi = [...phi_prev];
        let grad = this.grad(phi_prev, step_size);
        for (let i=0; i < phi.length; i++) {
          phi[i] -= a * exp(-0.2 * k) * grad[i];
          if (phi[i] > rot_max && i == 0) {phi[i] = rot_max;}
          if (phi[i] < rot_min && i == 0) {phi[i] = rot_min;}
        }
        if (arr_mag(grad) <= gradLimit) {
          console.log("Breaking")
          break;
        }
        phi_prev = phi;
      }
      console.log("iter lim")
      return phi;
  
    }
  
    update(newTarget) {
      if (!this._target.equals(newTarget)) {
        this._target = newTarget;
        this._prevSolution = [...this._pos];
        this._solution = this.gradDescent();
        
      }
    }

    linear_interp() {
        let zeta = 0.75;
        let w_n = 2;
        let gain = -0.1;
        let dt = 1/60;
        let k_1 = zeta / (PI * w_n);
        let k_2 = 1 / ((2 * PI * w_n) ** 2);
        let k_3 = (gain * zeta) / (2 * PI * w_n);

        for (let i=0; i<this._pos.length; i++) {
            let vel_in = (this._solution[i] - this._pos[i]) * dt;
            this._pos[i] += dt * this._vel[i];
            this._vel[i] += dt * (this._solution[i] + k_3 * vel_in - this._pos[i] - k_1 * this._vel[i]) / k_2;
            }
        }

  
    render() {
    colorMode("hsb")
      push();
      translate(this._origin.x - 0.5 * (this._lengths[0]/1.5), this._origin.y);
      stroke(360);
      fill(48, 80, 100);
      strokeWeight(3);
      //translate(0.5 * (-this._lengths[0]/1.5), 0);
  
      for (let i=0; i<this._lengths.length; i++) {
        stroke(360);
        strokeWeight(3);
        rotate(constrain_rot(this._pos[i]));
        
        let rect_wid = this._lengths[i]/4;
        rect(-rect_wid/2, 0, rect_wid, this._lengths[i], this._lengths[i]/20);
        stroke(0, 100, 100);
        if (i == this._lengths.length-1) {
          strokeWeight(5);
          line(0, this._lengths[i], 0, this._lengths[i]+30);
          line(0, this._lengths[i], -30, this._lengths[i]);
        }
        translate(0, this._lengths[i]);
        rotate(-1 * constrain_rot(this._pos[i]));
      }
      pop();
      push();
      stroke(360);
      fill(48, 80, 100);
      strokeWeight(3);
      translate(this._origin.x, this._origin.y);
      circle(0, 0, this._lengths[0]/1.5)
      pop();
    }
  }
  
  
  function arr_mag(array) {
    let cnt = 0
    for (let i = 0; i < array.length; i++) {
      cnt += array[i]**2;
    }
    return sqrt(cnt);
  }

  function euc_dist(arr_1, arr_2) {
    let dist = 0;
    for (let i=0; i<arr_1.length; i++) {
      dist += (arr_1[i] - arr_2[i])**2;
    }
    return dist;
  }

  function constrain_rot(angle) {
    return (angle + PI) % (2 * PI) - PI;
  }