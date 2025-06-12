/***

advanced-pid-controller/sample-application-template.js  Copyright 2025, Marc Alzen @ Rasche & Wessler GmbH.

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

// Import the PID Controller. Ensure the path is correct.
// If you renamed the project (e.g., 'rasche-wessler-pid-controller'), adjust this accordingly.
const PIDController = require('./index.js'); // Uses 'index.js' as it's the standard export.

// --- Simulation Parameters ---
const SIMULATION_DT_MS = 100; // Simulation step interval in milliseconds
const TOTAL_SIMULATION_TIME_MS = 35000; // Total simulation time in milliseconds (35 seconds)
let currentSimulationTimeMs = 0;

// --- Process Parameters (Example: Simple Heating System) ---
let currentTemperature = 20.0; // Current temperature in °C
const ROOM_TEMPERATURE = 20.0; // Ambient room temperature
const HEATING_RATE_FACTOR = 0.2; // Temperature change per unit of control output per second
const COOLING_RATE_FACTOR = 0.5; // Natural cooling per degree difference from room temp per second

// --- PID Controller Configuration ---
const Kp = 0.1;   // Proportional gain
const Ki = 0.5;  // Integral gain
const Kd = 0.1;   // Derivative gain
const CONTROLLER_DT_SEC = SIMULATION_DT_MS / 1000; // dt for the controller in seconds

// Controller output limits (e.g., heating power from 0% to 100%)
const OUTPUT_MIN = 0;
const OUTPUT_MAX = 100;

// Deadband: +/- tolerance around the setpoint within which the controller output is zero
const DEADBAND = 0.5; // +/- 0.5 °C

// Create a PID Controller instance with all new parameters
const controller = new PIDController(Kp, Ki, Kd, CONTROLLER_DT_SEC, OUTPUT_MIN, OUTPUT_MAX, DEADBAND);

console.log("--- PID Controller Simulation Start ---");
console.log(`Initial Temperature: ${currentTemperature.toFixed(2)} °C`);
console.log(`Controller dt: ${CONTROLLER_DT_SEC}s`);
console.log(`Output Limits: ${controller.outputMin} to ${controller.outputMax}`);
console.log(`Deadband: +/- ${controller.deadband} °C`);
console.log("-------------------------------------");

// --- Simulation Loop ---
const simulationInterval = setInterval(() => {
    currentSimulationTimeMs += SIMULATION_DT_MS;

    // --- Phase-based Target Changes & Feature Demonstrations ---
    if (currentSimulationTimeMs === 100) {
        controller.setTarget(25); // Phase 1: Initial target
        console.log(`\n\nTime: ${currentSimulationTimeMs / 1000}s - Phase 1: Target set to ${controller.target.toFixed(1)} °C.`);
    } else if (currentSimulationTimeMs === 10000) {
        controller.setTarget(95); // Phase 2: High target to demonstrate saturation and anti-windup
        console.log(`\n\nTime: ${currentSimulationTimeMs / 1000}s - Phase 2: Target set to ${controller.target.toFixed(1)} °C (Demonstrating Anti-Windup).`);
    } else if (currentSimulationTimeMs === 23000) {
        controller.setTarget(59.2); // Phase 3: Target close to current temp to demonstrate deadband
        console.log(`\n\nTime: ${currentSimulationTimeMs / 1000}s - Phase 3: Target set to ${controller.target.toFixed(1)} °C (Demonstrating Deadband).`);
        // Optionally, dynamically change deadband during runtime
        // controller.setDeadband(0.2);
        // console.log(`Deadband dynamically set to +/- ${controller.deadband} °C.`);
    } else if (currentSimulationTimeMs === 30000) {
        console.log(`\n\nTime: ${currentSimulationTimeMs / 1000}s - Demonstrating controller.reset().`);
        controller.reset(); // Demonstrates controller.reset()
        console.log("Controller state reset.");
        controller.setTarget(28); // Set a new target after reset
        console.log(`Target set to ${controller.target.toFixed(1)} °C after reset.`);
    }

    // Read the current process variable (PV) from the "sensor"
    const currentValue = currentTemperature;

    // Update the PID controller and get the control output
    const controlOutput = controller.update(currentValue);

    // --- Process Simulation ---
    // Calculate temperature increase from heater (based on control output)
    let tempChangeFromHeater = controlOutput * HEATING_RATE_FACTOR * CONTROLLER_DT_SEC;
    // Calculate natural cooling (system cools towards room temperature)
    let tempChangeFromCooling = (currentTemperature - ROOM_TEMPERATURE) * COOLING_RATE_FACTOR * CONTROLLER_DT_SEC;

    currentTemperature += tempChangeFromHeater - tempChangeFromCooling;

    let currentSimulationTimeSec = currentSimulationTimeMs / 1000
    // Log current state and PID components
    if (currentSimulationTimeSec % 1 === 0){
        console.log(
            `Time: ${currentSimulationTimeSec}.0s | Temp: ${currentTemperature.toFixed(2)} | Target: ${controller.target.toFixed(2)} | Output: ${controlOutput.toFixed(2)} | P:${controller.p.toFixed(2)} I:${controller.i.toFixed(2)} D:${controller.d.toFixed(2)}`
        );     
    } else {
        console.log(
            `Time: ${currentSimulationTimeSec}s | Temp: ${currentTemperature.toFixed(2)} | Target: ${controller.target.toFixed(2)} | Output: ${controlOutput.toFixed(2)} | P:${controller.p.toFixed(2)} I:${controller.i.toFixed(2)} D:${controller.d.toFixed(2)}`
        );  
    }


    // --- End Simulation ---
    if (currentSimulationTimeMs >= TOTAL_SIMULATION_TIME_MS) {
        clearInterval(simulationInterval);
        console.log("\n--- Simulation End ---");
        console.log("Note: Observe 'Output' and 'I:' (Integral component)");
        console.log("during saturation (e.g., Temp targets 40°C) and deadband demonstration (Temp around 21°C).");
        console.log("Also observe 'D:' during setpoint changes (no strong 'kick' due to PV derivative).");
        console.log("After the reset, I and D components should return to 0.");
    }
}, SIMULATION_DT_MS);