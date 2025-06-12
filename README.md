# PID Controller

This repository provides a highly robust and flexible Proportional-Integral-Derivative (PID) controller for Node.js environments. It originated as a fork of the `simple-pid-controller` project by hj91 and has been significantly extended with advanced features.

## New Features and Improvements

The following significant improvements have been added:

* **Deadband Functionality**: A configurable deadband (tolerance band) around the target value has been introduced. When the process variable is within this band, the controller output is set to zero, and integral/derivative histories are reset. This prevents unnecessary control actions and reduces system "chatter" when the system is close to the setpoint.

* **Anti-Windup Functionality**: The controller now prevents the integral component from growing uncontrollably when the controller's output reaches its predefined saturation limits. This ensures more stable control behavior, especially during large setpoint changes or disturbances.

* **Improved D-Component ("Derivative Kick" Avoidance)**: The derivative is now calculated based on the change in the process value (`currentValue`) instead of the change in error. This eliminates the undesirable "derivative kick" that can occur with sudden setpoint changes, leading to smoother control behavior.

* **Configurable Output Limits**: The controller can now be customized to specific output ranges using `outputMin` and `outputMax` in the constructor. By default, these are set to 0 and 100.

* **More Robust Validation**: Additional checks in the constructor ensure that the `dt`, `outputMin`, and `outputMax` parameters have valid values to prevent misconfigurations.

* **Reset Function**: A `reset()` method has been added to reset the internal state of the controller (integral and derivative components), which is useful for restarting the controller after manual interventions or interruptions.

* **Getters for P, I, and D Components**: Getters (`p`, `i`, `d`) have been added to allow retrieval of the individual components of the control signal at any time, facilitating analysis and debugging.

A Proportional-Integral-Derivative (PID) controller is a control loop feedback mechanism widely used in industrial control systems. This module provides a PID controller implementation in Node.js.

## Installation

To install this module, run the following command:

```sh
npm install advanced-pid-controller
```

## Usage

First, require the module:

```javascript
const PIDController = require('advanced-pid-controller');
```

Then, create a new PIDController instance. You can optionally provide proportional, integral, and derivative gains, as well as the output limits and deadband:

```javascript
// Example with default output limits (0 to 100) and default deadband (0.5)
const controller1 = new PIDController(1.2, 1, 0.01);

// Example with custom output limits (-50 to 50) and a custom deadband (1.0)
const controller2 = new PIDController(1.2, 1, 0.01, 1.0, -50.0, 50.0, 1.0);

```

You can set a new target for the controller:

```javascript
controller1.setTarget(34);
```

You can also dynamically adjust the deadband:

```javascript
controller1.setDeadband(0.2); // Set deadband to +/- 0.2

```

And you can update the controller with the current value to get the control output:

```javascript
let controlOutput = controller1.update(currentValue);
```

To reset the controller (e.g., after manual operation):

```javascript
controller1.reset();
```

You can also retrieve the individual components of the control signal:

```javascript
console.log(`P-Component: ${controller1.p}`);
console.log(`I-Component: ${controller1.i}`);
console.log(`D-Component: ${controller1.d}`);
```

## API

This module exports the `PIDController` class, which has the following methods and properties:

* `constructor(k_p = 1.0, k_i = 0.0, k_d = 0.0, dt = 1.0, outputMin = 0.0, outputMax = 100.0, deadband = 0.5)`: Constructs a new PIDController.

    * `k_p`: Proportional gain.

    * `k_i`: Integral gain.

    * `k_d`: Derivative gain.

    * `dt`: Time interval between updates (must be positive).

    * `outputMin`: Minimum possible controller output.

    * `outputMax`: Maximum possible controller output.

    * `deadband`: Tolerance band around the target within which output is zero (must be non-negative).

* `setTarget(target)`: Sets a new target for the controller.

* `update(currentValue)`: Updates the controller with the current process variable value and calculates the control output. Returns the clamped control output.

* `reset()`: Resets the internal state of the controller (integral and derivative components).

* `p` (Getter): Returns the current proportional component of the control signal.

* `i` (Getter): Returns the current integral component of the control signal.

* `d` (Getter): Returns the current derivative component of the control signal.

## Applications

PID controllers are used in a wide variety of applications in industrial control systems and other areas, including:

* Controlling the temperature of an oven

* Regulating the speed of a car

* Managing the flight controls of an airplane

* Controlling the power output of a generator

By using this module, developers can implement PID control in their Node.js applications without having to understand all of the underlying mathematics.

## License

This module is licensed under the **Apache License 2.0**.

The full text of the license is available in the `LICENSE` file in this repository.

## Authors and Maintainers

**Developed and Maintained by**: Marc Alzen @ Rasche & Wessler GmbH
    * This project is actively developed and maintained by Rasche & Wessler GmbH, led by Marc Alzen.

### Original Work & Contributions

**Based on Original Work by**: Harshad Joshi @ Bufferstack.IO Analytics Technology LLP, Pune
    * This project began as a fork of `simple-pid-controller` by Harshad Joshi (hj91). His foundational work provided the basis for this enhanced PID controller.
**Key Enhancements**: Anti-windup, improved D-component, configurable output limits, deadband functionality, and other enhancements.