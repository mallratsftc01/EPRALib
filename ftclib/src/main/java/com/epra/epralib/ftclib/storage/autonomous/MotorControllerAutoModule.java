package com.epra.epralib.ftclib.storage.autonomous;

import com.epra.epralib.ftclib.movement.MotorController;

/// A class that stores instructions for a [MotorController] during auto.
///
/// JSON Format:
/// <pre><code>
///         {
///             "id": string,
///             "target": int,
///             "tolerance": float,
///             "maxPower": float,
///             "power": float,
///         } </code></pre>
///
/// Intended to be used as a part of an [AutoStep].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see MotorController#moveToTarget(double, double, boolean)
public class MotorControllerAutoModule {

    private final String id;
    private final int target;
    private final double tolerance;
    private final double maxPower;
    private final double power;

    /// A class that stores instructions for a [MotorController] during auto.
    ///
    /// JSON Format:
    /// <pre><code>
    ///         {
    ///             "id": string,
    ///             "target": int,
    ///             "tolerance": float,
    ///             "maxPower": float,
    ///             "power": float,
    ///         } </code></pre>
    ///
    /// If `tolerance` is set to -1 the motor controller will run at `power`.
    /// Otherwise, `power` will not be used.
    ///
    /// Intended to be used as a part of an [AutoStep].
    /// @param id        The `id` of the motor controller this module contains instructions for
    /// @param target    The target in motor ticks of the motor controller
    /// @param tolerance The tolerance for reaching the target as a double between 0.0 and 1.0
    /// @param maxPower  The maximum absolute power the motor controller may reach. Capped at 1.0
    /// @param power     The power to run the motor controller at
    /// @see MotorController#moveToTarget(double, double, boolean)
    public MotorControllerAutoModule(String id, int target, double tolerance, double maxPower, double power) {
        this.id = id;
        this.target = target;
        this.tolerance = tolerance;
        this.maxPower = maxPower;
        this.power = power;
    }
    /// Returns the `id` of the [MotorController] this module contains instructions for.
    /// @return The `id` of the motor controller this module contains instructions for
    public String id() { return id; }
    /// Returns the target position for the [MotorController] in motor-specific ticks.
    /// @return The target position for the motor controller
    public int target() { return target; }
    /// Returns the tolerance for the [MotorController] reaching the `target`.
    /// @return The tolerance for the motor controller
    public double tolerance() { return tolerance; }
    /// Returns the maximum absolute power the [MotorController] may run at.
    /// @return The maximum absolute power
    public double maxPower() { return maxPower; }
    /// Returns the power the [MotorController] will use if [#tolerance()] is -1.
    /// @return The power to run the motor controller at
    public double power() { return power; }
}