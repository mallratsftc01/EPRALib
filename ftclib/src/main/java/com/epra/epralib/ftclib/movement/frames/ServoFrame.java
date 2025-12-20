package com.epra.epralib.ftclib.movement.frames;

import com.epra.epralib.ftclib.movement.Motor;
import com.qualcomm.robotcore.hardware.Servo;

/// A [Motor] wrapper for a [Servo].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class ServoFrame implements Motor{

    private final Servo servo;
    private boolean enabled;

    /// A [Motor] wrapper for a [Servo].
    /// @param servo The servo to be wrapped
    public ServoFrame(Servo servo) {
        this.servo = servo;
        this.enabled = true;
    }

    /// {@inheritDoc}
    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction == Direction.FORWARD ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
    }
    /// {@inheritDoc}
    @Override
    public Direction getDirection() {
        return (servo.getDirection() == Servo.Direction.FORWARD) ? Direction.FORWARD : Direction.REVERSE;
    }

    /// {@inheritDoc}
    @Override
    public boolean isEnabled() { return enabled; }
    /// {@inheritDoc}
    @Override
    public void setEnabled() { enabled = true; }
    /// {@inheritDoc}
    @Override
    public void setDisabled() { enabled = false; }

    /// The amount of power sent to a [Servo] cannot be controlled or monitored.
    /// @return `False`
    public boolean powerEnabled() { return false; }
    /// Has no effect as the amount of power sent to a [Servo] cannot be controlled.
    @Override
    public void setPower(double power) {}
    /// Returns `NaN` as the amount of power sent to a [Servo] cannot be monitored.
    /// @return `NaN`
    @Override
    public double getPower() { return Double.NaN; }

    /// A [Servo] can monitor its position.
    /// @return `True`
    @Override
    public boolean positionMonitoringEnabled() { return true; }
    /// A [Servo] can control its position.
    /// @return `True`
    @Override
    public boolean positionControlEnabled() { return true; }
    /// Returns the current rotational position of the motor's axle.
    ///
    /// The position will be a number between 0 and 1.
    /// @return The current position of this motor
    @Override
    public double getCurrentPosition() {
        return servo.getPosition();
    }

    /// Rotates the motor to a specified rotational position of the motor's axle.
    ///
    /// A [Servo] can rotate between position 0 and position 1.
    /// Any position more than 1 will be treated as 1, and any position less than 0 will be treated as 0.
    /// @param position The position to rotate the motor to
    /// @return Whether the motor has successfully reached the target position
    public boolean setPosition(double position) {
        if (enabled) servo.setPosition(position);
        return position == getCurrentPosition();
    }

    /// Returns the [Servo] object contained within this wrapper.
    /// @return The wrapped servo
    @Override
    public Object getSelf() {
        return servo;
    }
    /// Returns the device name of the wrapped [Servo].
    /// @return The motor's device name
    @Override
    public String toString() { return servo.getDeviceName(); }
}
