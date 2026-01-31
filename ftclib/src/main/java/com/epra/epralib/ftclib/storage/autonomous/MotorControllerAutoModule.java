package com.epra.epralib.ftclib.storage.autonomous;

import com.epra.epralib.ftclib.movement.MotorController;

/// A class that stores instructions for a [MotorController] during auto.
///
/// JSON Format:
/// <pre><code>
///         {
///             "id": string,
///             "motorMode": string,
///             "target": int/string,
///             "tolerance": float/string,
///             "maxPower": float/string
///         } </code></pre>
///
/// A [MotorController] auto module can use one of three different `motorMode`s:
///
/// - `none` will not drive the motor at all, setting the power to 0.
///
/// - `direct_power` moves the motor at `maxPower` constantly using [MotorController#setPower(double)].
///  `maxPower` should be between -1 and 1.
///
/// - `targeted` moves the motor to `target` in motor-specific ticks using [MotorController#moveToTarget(double, double, boolean)].
/// The absolute maximum power will be `maxPower`.
/// The tolerance for reaching `target` is `tolerance` (1 being no precision, and 0 being infinite precision).
///
/// Intended to be used as a part of an [AutoStep].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see MotorController#moveToTarget(double, double, boolean)
public class MotorControllerAutoModule {

    private final String id;
    private String motorMode;
    private final AutoProgram.AutoModuleDataSupplier target;
    private final AutoProgram.AutoModuleDataSupplier tolerance;
    private final AutoProgram.AutoModuleDataSupplier maxPower;

    /// A class that stores instructions for a [MotorController] during auto.
    ///
    /// JSON Format:
    /// <pre><code>
    ///         {
    ///             "id": string,
    ///             "motorMode": string,
    ///             "target": int,
    ///             "tolerance": float,
    ///             "maxPower": float
    ///         } </code></pre>
    ///
    /// A [MotorController] auto module can use one of three different `motorMode`s:
    ///
    /// - `none` will not drive the motor at all, setting the power to 0.
    ///
    /// - `direct_power` moves the motor at `maxPower` constantly. `maxPower` should be between -1 and 1.
    ///
    /// - `targeted` moves the motor to `target` in motor-specific ticks. The absolute maximum power will be
    /// `maxPower`. The tolerance for reaching `target` is `tolerance` (1 being no precision, and 0 being infinite precision).
    ///
    /// Intended to be used as a part of an [AutoStep].
    /// @param id        The `id` of the motor controller this module contains instructions for
    /// @param motorMode The mode the motor will be powered according to
    /// @param target    The target in motor ticks of the motor controller
    /// @param tolerance The tolerance for reaching the target as a double between 0.0 and 1.0
    /// @param maxPower  The maximum absolute power the motor controller may reach. Capped at 1.0
    /// @see MotorController#moveToTarget(double, double, boolean)
    public MotorControllerAutoModule(String id, String motorMode, int target, double tolerance, double maxPower) {
        this.id = id;
        this.motorMode = motorMode;
        this.target = new AutoProgram.AutoModuleDataSupplierNumber(target);
        this.tolerance = new AutoProgram.AutoModuleDataSupplierNumber(tolerance);
        this.maxPower = new AutoProgram.AutoModuleDataSupplierNumber(maxPower);
    }

    /// A class that stores instructions for a [MotorController] during auto.
    ///
    /// JSON Format:
    /// <pre><code>
    ///         {
    ///             "id": string,
    ///             "motorMode": string,
    ///             "target": int/string,
    ///             "tolerance": float/string,
    ///             "maxPower": float/string
    ///         } </code></pre>
    /// Data can be strings that can reference [AutoProgram] data suppliers and are parsed using
    /// [AutoProgram#parseArithmatic(String)].
    ///
    /// A [MotorController] auto module can use one of three different `motorMode`s:
    ///
    /// - `none` will not drive the motor at all, setting the power to 0.
    ///
    /// - `direct_power` moves the motor at `maxPower` constantly. `maxPower` should be between -1 and 1.
    ///
    /// - `targeted` moves the motor to `target` in motor-specific ticks. The absolute maximum power will be
    /// `maxPower`. The tolerance for reaching `target` is `tolerance` (between 0 and 1).
    ///
    /// Intended to be used as a part of an [AutoStep].
    /// @param id        The `id` of the motor controller this module contains instructions for
    /// @param motorMode The mode the motor will be powered according to
    /// @param target    The target in motor ticks of the motor controller
    /// @param tolerance The tolerance for reaching the target as a double between 0.0 and 1.0
    /// @param maxPower  The maximum absolute power the motor controller may reach. Capped at 1.0
    /// @see MotorController#moveToTarget(double, double, boolean)
    public MotorControllerAutoModule(String id, String motorMode,
                                     AutoProgram.AutoModuleDataSupplier target,
                                     AutoProgram.AutoModuleDataSupplier tolerance,
                                     AutoProgram.AutoModuleDataSupplier maxPower) {
        this.id = id;
        this.motorMode = motorMode;
        this.target = target;
        this.tolerance = tolerance;
        this.maxPower = maxPower;
    }
    /// Returns the `id` of the [MotorController] this module contains instructions for.
    /// @return The `id` of the motor controller this module contains instructions for
    public String id() { return id; }

    /// Returns the `motorMode` of this [MotorController] auto module.
    ///
    /// A [MotorController] auto module can use one of three different `motorMode`s:
    ///
    /// - `none` will not drive the motor at all, setting the power to 0.
    ///
    /// - `direct_power` moves the motor at `maxPower` constantly. `maxPower` should be between -1 and 1.
    ///
    /// - `targeted` moves the motor to `target` in motor-specific ticks. The absolute maximum power will be
    /// `maxPower`. The tolerance for reaching `target` is `tolerance` (between 0 and 1).
    /// @return The `motorMode` of this motor controller auto module
    public String motorMode() { return motorMode; }
    /// Returns the target position for the [MotorController] in motor-specific ticks.
    /// @return The target position for the motor controller
    public int target() { return (int) target.get(); }
    /// Returns the tolerance for the [MotorController] reaching the `target`.
    /// @return The tolerance for the motor controller
    public double tolerance() { return tolerance.get(); }
    /// Returns the maximum absolute power the [MotorController] may run at.
    /// @return The maximum absolute power
    public double maxPower() { return maxPower.get(); }

    public boolean equals(MotorControllerAutoModule other) {
        return id.equals(other.id) &&
                motorMode.equals(other.motorMode) &&
                target == other.target &&
                tolerance == other.tolerance &&
                maxPower == other.maxPower;
    }
}