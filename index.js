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
   * @param {number} [k_i=0.1] - Integral gain.
   * @param {number} [k_d=0.3] - Derivative gain.
   * @param {number} [dt=1.0] - Time interval between updates.
   * @param {number} [outputMin=0.0] - Minimum possible controller output.
   * @param {number} [outputMax=100.0] - Maximum possible controller output.
   * @param {number} [deadband=0.5] - Tolerance band around the target within which output is paused.
   */
  constructor(k_p = 0.15, k_i = 0.1, k_d = 0.03, dt = 1.0, outputMin = 0.0, outputMax = 100.0, deadband = 0.5) {
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
    this.currentValue = 0;      // Initialisiert den aktuellen Wert auf 0
    this.sumError = 0;
    this.lastError = 0;         // Stores error from previous update (if error-based D is used)
    this.previousCurrentValue = 0; // Speichert den Wert vom vorherigen Update für die D-Berechnung. Initial auf 0.
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
   * These getters now return the internally stored calculated values.
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

    // Der aktuelle PV-Wert für diese Iteration
    const currentProcessValue = currentValue; 
    const error = this.target - currentProcessValue;

    // --- Deadband Functionality (Pausing the controller) ---
    // Wenn der Prozesswert im Deadband ist, werden alle Regelberechnungen pausiert.
    // Der Output und die Komponentenwerte behalten ihren letzten aktiven Zustand bei.
    if (this.deadband > 0 && Math.abs(error) <= this.deadband) {
        // Der D-Anteil muss wieder aktiv werden, sobald das Deadband verlassen wird.
        // Dafür muss `_isFirstActiveUpdate` auf false gesetzt werden.
        // WICHTIG: `previousCurrentValue` wird im Deadband NICHT aktualisiert,
        // da es den Wert vom letzten AKTIVEN Schritt halten soll.
        if (this._isFirstActiveUpdate === false) { // Wenn wir bereits aktiv waren, aber jetzt ins Deadband gehen
            this._isFirstActiveUpdate = true; // Setze Flag zurück, damit D-Anteil bei Wiederaufnahme des Reglers
                                              // nicht sofort einen Kick verursacht, falls der Wert stark springt.
                                              // (Alternative: previousCurrentValue = currentProcessValue HIER setzen)
                                              // Die hier gewählte Methode ist sicherer, um den D-Kick bei Wiedereinstieg zu vermeiden.
        }
        return this.y; // Gebe den letzten aktiven Output zurück
    }

    // --- Normal PID Calculation (if outside deadband) ---

    // Setze das Flag zurück, da wir jetzt aktiv sind.
    this._isFirstActiveUpdate = false; 

    // Calculate P-component
    this._p_val = this.k_p * error;

    // Accumulate error for Integral term
    this.sumError += error * this.dt;
    this._i_val = this.k_i * this.sumError;

    // Calculate D-component
    // WICHTIG: D-Anteil nur berechnen, wenn dies NICHT der erste aktive Durchlauf ist
    // seit dem Start oder Reset des Reglers. Dies verhindert den "D-Kick".
    if (this.dt === 0) { // Sicherheitscheck
        this._d_val = 0;
    } else if (this.previousCurrentValue === 0 && currentProcessValue !== 0) { // Spezieller Fall für den allerersten "echten" Wert
        // Wenn previousCurrentValue noch der Initialwert (0) ist und der aktuelle PV nicht 0 ist,
        // gehen wir davon aus, dass dies der erste "sinnvolle" Wert ist und setzen D auf 0.
        this._d_val = 0;
    } else {
        // D-Anteil basierend auf der Änderung des Prozesswertes (aktuell - vorherig)
        this._d_val = this.k_d * (currentProcessValue - this.previousCurrentValue) / this.dt;
    }
    
    // Speichere den aktuellen Prozesswert als "vorherigen" Wert für die D-Berechnung im nächsten Iterationszyklus.
    this.previousCurrentValue = currentProcessValue;


    // Control value calculation: Sum of P, I, D components
    this.y = this._p_val + this._i_val + this._d_val;

    // --- Anti-windup functionality and Output Clamping ---
    // Dies verhindert, dass der Integralanteil unkontrolliert anwächst, wenn der Output gesättigt ist.
    if (this.y >= this.outputMax) {
      this.y = this.outputMax;
      // Passe sumError an, wenn der Integralanteil zur Sättigung über outputMax beiträgt
      if (this.k_i > 0 && this._i_val > (this.outputMax - (this._p_val + this._d_val))) {
          this.sumError = (this.outputMax - (this._p_val + this._d_val)) / this.k_i;
      }
    } else if (this.y <= this.outputMin) {
      this.y = this.outputMin;
      // Passe sumError an, wenn der Integralanteil zur Sättigung unter outputMin beiträgt
      if (this.k_i > 0 && this._i_val < (this.outputMin - (this._p_val + this._d_val))) {
          this.sumError = (this.outputMin - (this._p_val + this._d_val)) / this.k_i;
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
    this.lastError = 0; // Reset lastError (though D-getter uses PV)
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