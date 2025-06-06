/***

simple-pid-controller/index.js  Copyright 2023, Harshad Joshi and Bufferstack.IO Analytics Technology LLP. Pune
                                Copyright 2024, Marc Alzen @ Rasche und Wessler GmbH.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

***/



"use strict";

/**
 * PID Controller.
 */
class PIDController {
  /**
   * Construct a PID Controller.
   *
   * @param {number} [k_p=1.0] - Proportional gain.
   * @param {number} [k_i=0.0] - Integral gain.
   * @param {number} [k_d=0.0] - Derivative gain.
   * @param {number} [dt=1.0] - Time interval between updates.
   * @param {number} [outputMin=0.0] - Minimum possible controller output.
   * @param {number} [outputMax=100.0] - Maximum possible controller output.
   * @param {number} [deadband=0.5] - Tolerance band around the target within which output is zero.
   */
  constructor(k_p = 1.0, k_i = 0.0, k_d = 0.0, dt = 1.0, outputMin = 0.0, outputMax = 100.0, deadband = 0.5) {
    // Validate constructor parameters
    if (typeof k_p !== 'number' || typeof k_i !== 'number' || typeof k_d !== 'number' || typeof dt !== 'number' || typeof outputMin !== 'number' || typeof outputMax !== 'number' || typeof deadband !== 'number') {
      throw new Error('PID Controller constructor parameters must all be numbers');
    }
    if (dt <= 0) {
        // dt must be positive to avoid division by zero and ensure proper time progression
        throw new Error('Time interval (dt) must be a positive number');
    }
    if (outputMin >= outputMax) {
        throw new Error('outputMin must be less than outputMax');
    }
    if (deadband < 0) {
        throw new Error('Deadband must be a non-negative number');
    }

    this.k_p = k_p;
    this.k_i = k_i;
    this.k_d = k_d;
    this.dt = dt;

    this.outputMin = outputMin; // Configurable minimum output
    this.outputMax = outputMax; // Configurable maximum output
    this.deadband = deadband;   // Tolerance band around target

    this.target = 0;
    this.currentValue = 0;
    this.sumError = 0;
    this.lastError = 0; // Stores error from previous update for derivative calculation (if used)
    this.previousCurrentValue = 0; // Stores currentValue from previous update for derivative calculation (avoids derivative kick)
    this.y = 0; // Current controller output

    // Define getters for P, I, and D values
    this.defineGetters();
  }

  /**
   * Define getters for P, I, and D components.
   */
  defineGetters() {
    Object.defineProperty(this, 'p', {
      get: function() {
        return this.k_p * (this.target - this.currentValue);
      }
    });

    Object.defineProperty(this, 'i', {
      get: function() {
        // If k_i is 0, the integral term is 0. Avoid division by zero.
        if (this.k_i === 0) {
          return 0;
        }
        // Invert the k_i value so that the control value calculation below is simplified.
        // This means 'i' represents the integral term *before* multiplication by k_i,
        // which is then effectively applied in the anti-windup logic.
        return this.sumError / this.k_i;
      }
    });

    Object.defineProperty(this, 'd', {
      get: function() {
        // If k_d is 0, the derivative term is 0. Avoid division by zero.
        if (this.k_d === 0 || this.dt === 0) {
          return 0;
        }
        // Calculate derivative based on change in process variable (currentValue)
        // This helps to avoid "derivative kick" when the setpoint (target) changes suddenly.
        return this.k_d * (this.currentValue - this.previousCurrentValue) / this.dt;
      }
    });
  }

  /**
   * Set a new target for the controller.
   *
   * @param {number} target - New target value.
   */
  setTarget(target) {
    if (typeof target !== 'number') {
      throw new Error('Target must be a number');
    }
    this.target = target;
  }

  /**
   * Set a new deadband value for the controller.
   *
   * @param {number} deadband - New non-negative deadband value.
   */
  setDeadband(deadband) {
    if (typeof deadband !== 'number' || deadband < 0) {
      throw new Error('Deadband must be a non-negative number');
    }
    this.deadband = deadband;
  }

  /**
   * Update the controller with the current value and calculate control output.
   *
   * @param {number} currentValue - Current process variable value.
   * @return {number} Controller output.
   */
  update(currentValue) {
    if (typeof currentValue !== 'number') {
      throw new Error('Current value must be a number');
    }

    this.previousCurrentValue = this.currentValue; // Store current value before updating for derivative
    this.currentValue = currentValue;

    const error = this.target - this.currentValue;

    // Apply Deadband functionality: If current value is within the deadband of the target,
    // the controller output is zero, and integral/derivative histories are reset to prevent
    // sudden output changes when exiting the deadband.
    if (this.deadband > 0 && Math.abs(error) <= this.deadband) {
        this.sumError = 0; // Reset integral component
        this.lastError = 0; // Reset last error for derivative (even if D-getter uses PV)
        this.previousCurrentValue = this.currentValue; // Prevent D-spike when exiting deadband
        this.y = 0; // Output is zero within deadband
        return this.y;
    }

    // If outside deadband, proceed with normal PID calculation
    this.sumError += error * this.dt;

    this.lastError = error; // Update lastError for next iteration

    // Control value calculation
    // Note: this.p, this.i, this.d are getters and are calculated dynamically here.
    this.y = this.p + this.i + this.d;

    // Anti-windup function for the I component and output clamping
    if (this.y >= this.outputMax) {
      this.y = this.outputMax;
      // Adjust sumError to prevent integral windup when output is saturated at max.
      if (this.k_i > 0) { // Only adjust if integral gain is positive and contributes to windup
        this.sumError = (this.outputMax - (this.p + this.d)) * this.k_i;
      } else {
        // If k_i is 0 or negative, integral term is not causing positive windup in this context.
        this.sumError = 0.0;
      }
    } else if (this.y <= this.outputMin) {
      this.y = this.outputMin;
      // Adjust sumError to prevent integral windup when output is saturated at min.
      // Using a small epsilon (0.01) if outputMin is 0, to allow integral to pull slightly.
      const integralTarget = this.outputMin === 0 ? 0.01 : this.outputMin;
      if (this.k_i > 0) { // Only adjust if integral gain is positive and contributes to windup
        this.sumError = (integralTarget - (this.p + this.d)) * this.k_i;
      } else {
        // If k_i is 0 or negative, integral term is not causing negative windup in this context.
        this.sumError = 0.0;
      }
    }

    return this.y;
  }

  /**
   * Reset function to restart the controller at 0, e.g. after manual operation or an interruption.
   */
  reset() {
    this.sumError = 0;
    this.lastError = 0;
    this.previousCurrentValue = 0; // Also reset previous value on reset
    this.y = 0; // Ensure output is reset as well
  }
}

module.exports = PIDController;
