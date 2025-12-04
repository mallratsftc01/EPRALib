package com.epra.epralib.ftclib.movement;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/// A [Motor] wrapper for a [DcMotorEx].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class DcMotorExFrame implements Motor {
    private final DcMotorEx motor;

    /// A motor wrapper for a [DcMotorEx].
    /// @param motor The DcMotorEx to wrap
    public DcMotorExFrame(DcMotorEx motor) { this.motor = motor; }

    /// {@inheritDoc}
    @Override
    public void setDirection(Direction direction) {
        switch (direction) {
            case FORWARD -> motor.setDirection(DcMotorSimple.Direction.FORWARD);
            case REVERSE -> motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    /// {@inheritDoc}
    @Override
    public Direction getDirection() {
        return switch (motor.getDirection()) {
            case FORWARD -> Direction.FORWARD;
            case REVERSE -> Direction.REVERSE;
        };
    }
    /// {@inheritDoc}
    @Override
    public boolean isEnabled() { return motor.isMotorEnabled(); }
    /// {@inheritDoc}
    @Override
    public void setEnabled() { motor.setMotorEnable(); }
    /// {@inheritDoc}
    @Override
    public void setDisabled() { motor.setMotorDisable(); }

    /// A [DcMotorEx] can control and monitor its power.
    /// @return `True`
    @Override
    public boolean powerEnabled() { return true; }
    /// Sets the amount of power to send the motor as a float between 1 (full power forward) and -1 (full power backwards).
    ///
    /// Power is typically proportional to the rotational velocity of the motor. A power of 0 will brake the motor.
    /// @param power Power, as a double between -1 and 1
    @Override
    public void setPower(double power) { motor.setPower(power); }
    /// Returns the amount of power being sent to the motor as a float between 1 (full power forward) and -1 (full power backwards).
    ///
    /// Power is typically proportional to the rotational velocity of the motor. A power of 0 will brake the motor.
    /// @return The current power of this motor
    @Override
    public double getPower() { return motor.getPower(); }

    /// A [DcMotorEx] can monitor its axle position.
    /// @return `True`
    @Override
    public boolean positionMonitoringEnabled() { return true; }
    /// A [DcMotorEx] cannot directly control its axle position.
    /// @return `False`
    @Override
    public boolean positionControlEnabled() { return false; }

    /// Returns the current rotational position of the motor's axle.
    ///
    /// A [DcMotorEx] will return position as an integer number of ticks.
    /// The number of ticks in one revolution varies by motor, but should be listed on the motor's product page.
    ///
    /// - A [REV HD Hex Motor](https://www.revrobotics.com/REV-41-1291/) with no gearboxes will have 28 ticks per revolution.
    /// - A [REV Core Hex Motor](https://www.revrobotics.com/rev-41-1300/) will have 288 ticks per revolution.
    /// @return The current position of this motor
    @Override
    public double getCurrentPosition() { return motor.getCurrentPosition(); }
    /// Has no effect as a [DcMotorEx] cannot directly set its position.
    /// @return `False`
    @Override
    public boolean setPosition(double position) { return false; }

    /// Returns the [DcMotorEx] object contained within this wrapper.
    /// @return The wrapped motor
    @Override
    public Object getSelf() { return motor; }

    /// Returns the device name of the wrapped [DcMotorEx].
    /// @return The motor's device name
    @Override
    public String toString() { return motor.getDeviceName(); }
}
