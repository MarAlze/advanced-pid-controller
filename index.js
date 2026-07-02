/***
advanced-pid-controller/index.js  Copyright 2025, Marc Alzen @ Rasche & Wessler GmbH.

This project began as a fork of `simple-pid-controller` by Harshad Joshi (hj91). His foundational work provided the basis for this enhanced PID controller.                          

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
   * @param {number} [k_p=0.15] - Proportional gain.
   * @param {number} [k_i=0.1] - Integral gain (or integration time TN in seconds if useCodesysI is true).
   * @param {number} [k_d=0.03] - Derivative gain.
   * @param {number} [dt=1.0] - Time interval between updates.
   * @param {number} [outputMin=0.0] - Minimum possible controller output.
   * @param {number} [outputMax=100.0] - Maximum possible controller output.
   * @param {number} [deadband=0.5] - Tolerance band around the target within which output is paused.
   * @param {boolean} [useCodesysI=false] - If true, integral part behaves like a Codesys PID controller.
   * @param {number} [iClamp=0] - Maximum absolute value for the integral term (0 to disable clamping).
   */
  constructor(k_p = 0.15, k_i = 0.1, k_d = 0.03, dt = 1.0, outputMin = 0.0, outputMax = 100.0, deadband = 0.5, useCodesysI = false, iClamp = 0) {
    // Validate constructor parameters
    if (typeof k_p !== 'number' || typeof k_i !== 'number' || typeof k_d !== 'number' || typeof dt !== 'number' || typeof outputMin !== 'number' || typeof outputMax !== 'number' || typeof deadband !== 'number' || typeof iClamp !== 'number') {
      throw new Error('PID Controller constructor parameters must all be numbers');
    }
    if (typeof useCodesysI !== 'boolean') {
      throw new Error('useCodesysI parameter must be a boolean');
    }
    if (iClamp < 0) {
      throw new Error('iClamp must be a non-negative number');
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
    this.useCodesysI = useCodesysI;
    this.iClamp = iClamp;

    this.outputMin = outputMin; // Configurable minimum output
    this.outputMax = outputMax; // Configurable maximum output
    this.deadband = deadband;   // Tolerance band around target

    this.target = 0;
    this.currentValue = 0;      // Initializes current value to 0
    this.sumError = 0;
    this.lastError = 0;         // Stores error from previous update (used for trapezoidal integration)
    this.previousCurrentValue = 0; // Stores the value from the previous update for D calculation. Initialized to 0.
    this.y = 0;                 // Current controller output

    // Internal variables to hold the calculated component values for getters
    // These will hold the last computed values when controller is paused.
    this._p_val = 0;
    this._i_val = 0;
    this._d_val = 0;

    // Flag to skip derivative calculation on the very first active update
    // This prevents a large D-kick if previousValue is uninitialized or 0.
    this._isFirstActiveUpdate = true;

    // Define getters for P, I, and D values
    this.defineGetters();
  }

  /**
   * Define getters for P, I, and D components.
   * These getters return the internally stored calculated values.
   */
  defineGetters() {
    Object.defineProperty(this, 'p', {
      get: function() {
        return this._p_val;
      }
    });

    Object.defineProperty(this, 'i', {
      get: function() {
        return this._i_val;
      }
    });

    Object.defineProperty(this, 'd', {
      get: function() {
        return this._d_val;
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

    // The current PV value for this iteration
    const currentProcessValue = currentValue; 
    const error = this.target - currentProcessValue;

    // --- Deadband Functionality (Pausing the controller) ---
    // If the process value is within the deadband, all controller calculations are paused.
    // The output and component values retain their last active state.
    if (this.deadband > 0 && Math.abs(error) <= this.deadband) {
        // The D-component must become active again as soon as the deadband is exited.
        // For this, `_isFirstActiveUpdate` must be set to true.
        // IMPORTANT: `previousCurrentValue` is NOT updated in the deadband,
        // because it should hold the value from the last ACTIVE step.
        if (this._isFirstActiveUpdate === false) { // If we were active before, but now enter the deadband
            this._isFirstActiveUpdate = true; // Reset flag so D-component doesn't cause a kick on re-entry
        }
        return this.y; // Return the last active output
    }

    // --- Normal PID Calculation (if outside deadband) ---

    const isFirstActive = this._isFirstActiveUpdate;
    // Reset the first update flag as we are now active.
    this._isFirstActiveUpdate = false; 

    // Calculate P-component
    this._p_val = this.k_p * error;

    // Run Integrator
    if (this.useCodesysI) {
      if (this.k_i === 0) {
        this._i_val = 0;
        this.sumError = 0;
      } else {
        if (isFirstActive) {
          this.lastError = error;
        }
        // Trapezoidal integration rule (Codesys compliant)
        const increment = (error + this.lastError) * 0.5 * this.dt;
        this.sumError += increment;
        if (Math.abs(this.k_p) > 1e-15) {
          this._i_val = (this.k_p / this.k_i) * this.sumError;
        } else {
          this._i_val = 0;
        }
      }
      this.lastError = error; // Store error for the next iteration
    } else {
      // Standard rectangular integration
      this.sumError += error * this.dt;
      this._i_val = this.k_i * this.sumError;
    }

    // Apply integral clamp if specified (iClamp > 0)
    if (this.iClamp > 0) {
      if (this._i_val > this.iClamp) {
        this._i_val = this.iClamp;
        if (this.useCodesysI) {
          if (this.k_i > 0 && Math.abs(this.k_p) > 1e-15) {
            this.sumError = this.iClamp * this.k_i / this.k_p;
          }
        } else {
          if (this.k_i !== 0) {
            this.sumError = this.iClamp / this.k_i;
          }
        }
      } else if (this._i_val < -this.iClamp) {
        this._i_val = -this.iClamp;
        if (this.useCodesysI) {
          if (this.k_i > 0 && Math.abs(this.k_p) > 1e-15) {
            this.sumError = -this.iClamp * this.k_i / this.k_p;
          }
        } else {
          if (this.k_i !== 0) {
            this.sumError = -this.iClamp / this.k_i;
          }
        }
      }
    }

    // Calculate D-component
    // IMPORTANT: Only calculate D-component if this is NOT the first active run
    // since the controller started or reset. This prevents the "D-kick".
    if (this.dt === 0) { // Safety check
        this._d_val = 0;
    } else if (this.previousCurrentValue === 0 && currentProcessValue !== 0) { // Special case for the very first "real" value
        // If previousCurrentValue is still the initial value (0) and current PV is not 0,
        // we assume this is the first meaningful value and set D to 0.
        this._d_val = 0;
    } else {
        // D-component based on the change of the process value (current - previous)
        this._d_val = this.k_d * (currentProcessValue - this.previousCurrentValue) / this.dt;
    }
    
    // Store the current process value as "previous" value for the D-calculation in the next iteration.
    this.previousCurrentValue = currentProcessValue;


    // Control value calculation: Sum of P, I, D components
    this.y = this._p_val + this._i_val + this._d_val;

    // --- Anti-windup functionality and Output Clamping ---
    // This prevents the integral term from accumulating uncontrollably when the output is saturated.
    if (this.y >= this.outputMax) {
      this.y = this.outputMax;
      // Adjust sumError if the integral term contributes to the saturation above outputMax
      if (this.useCodesysI) {
        if (this.k_i > 0 && Math.abs(this.k_p) > 1e-15 && this._i_val > (this.outputMax - (this._p_val + this._d_val))) {
          this.sumError = (this.outputMax - (this._p_val + this._d_val)) * this.k_i / this.k_p;
        }
      } else {
        if (this.k_i > 0 && this._i_val > (this.outputMax - (this._p_val + this._d_val))) {
          this.sumError = (this.outputMax - (this._p_val + this._d_val)) / this.k_i;
        }
      }
    } else if (this.y <= this.outputMin) {
      this.y = this.outputMin;
      // Adjust sumError if the integral term contributes to the saturation below outputMin
      if (this.useCodesysI) {
        if (this.k_i > 0 && Math.abs(this.k_p) > 1e-15 && this._i_val < (this.outputMin - (this._p_val + this._d_val))) {
          this.sumError = (this.outputMin - (this._p_val + this._d_val)) * this.k_i / this.k_p;
        }
      } else {
        if (this.k_i > 0 && this._i_val < (this.outputMin - (this._p_val + this._d_val))) {
          this.sumError = (this.outputMin - (this._p_val + this._d_val)) / this.k_i;
        }
      }
    }

    return this.y;
  }

  /**
   * Reset function to restart the controller at 0, e.g. after manual operation or an interruption.
   * Clears all internal accumulated states and resets component values.
   */
  reset() {
    this.sumError = 0;
    this.lastError = 0; // Reset lastError
    this.currentValue = 0; // Set current value to a neutral state
    this.previousCurrentValue = 0; // Reset previous value for D-calculation
    this.y = 0; // Ensure output is reset as well
    this._p_val = 0; // Reset displayed P-component
    this._i_val = 0; // Reset displayed I-component
    this._d_val = 0; // Reset displayed D-component
    this._isFirstActiveUpdate = true; // Set flag to true for the next active update
  }
}

module.exports = PIDController;