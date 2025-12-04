package com.epra.epralib.ftclib.movement;

import androidx.annotation.NonNull;

/// An interface for generalizing motors of all types via wrapping.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public interface Motor {
    /// The logical direction for a motor to operate in.
    enum Direction {
        FORWARD,
        REVERSE
    }

    /// Sets the [Direction] in which this motor operates.
    /// @param direction A direction
    void setDirection(Direction direction);

    /// Returns the current [Direction] in which this motor is set as operating.
    /// @return The current direction of the motor
    Direction getDirection();

    /// Returns whether this motor is energized.
    ///
    /// If this motor is not energized, all commands sent to it will be ignored until it is reenergized.
    /// **Warning: This may result in the motor attempting several actions at once upon reenergization, which could result
    /// in system errors.**
    /// @return If this motor is energized
    boolean isEnabled();
    /// Energizes this motor.
    void setEnabled();
    /// De-energizes this motor.
    ///
    /// All commands sent to it will be ignored until it is reenergized.
    /// **Warning: This may result in the motor attempting several actions at once upon reenergization, which could result
    /// in system errors.**
    void setDisabled();

    /// Indicates whether the amount of power sent to the motor can be controlled or monitored.
    /// @return If power is enabled for this motor
    boolean powerEnabled();
    /// Sets the amount of power to send the motor as a float between 1 (full power forward) and -1 (full power backwards).
    ///
    /// Power is typically proportional to the rotational velocity of the motor. A power of 0 will brake the motor.
    /// If [#powerEnabled()] is false, this will have no effect.
    /// @param power Power, as a double between -1 and 1
    void setPower(double power);
    /// Returns the amount of power being sent to the motor as a float between 1 (full power forward) and -1 (full power backwards).
    ///
    /// Power is typically proportional to the rotational velocity of the motor. A power of 0 will brake the motor.
    /// If [#powerEnabled()] is false, this will instead return `NaN`.
    /// @return The current power of this motor
    double getPower();

    /// Indicates if the rotational position of the motor's axle can be monitored.
    /// @return If position monitoring is enabled for this motor
    boolean positionMonitoringEnabled();
    /// Indicates if the rotational position of the motor's axle can be controlled.
    /// @return If position control is enabled for this motor
    boolean positionControlEnabled();
    /// Returns the current rotational position of the motor's axle.
    ///
    /// The units of this position are specific to the motor.
    /// If [#positionMonitoringEnabled()] is `False`, this will instead return `NaN`.
    /// @return The current position of this motor
    double getCurrentPosition();
    /// Rotates the motor to a specified rotational position of the motor's axle.
    ///
    /// The units of this position are specific to the motor.
    /// If [#positionControlEnabled()] is false, this will have no effect and return `False`.
    /// @param position The position to rotate the motor to
    /// @return Whether the motor has successfully reached the target position
    boolean setPosition(double position);

    /// Returns the motor object contained within this wrapper.
    /// @return The wrapped motor
    Object getSelf();
    /// Returns the identification of the wrapped motor.
    /// @return The motor's identification
    @NonNull
    String toString();
}
