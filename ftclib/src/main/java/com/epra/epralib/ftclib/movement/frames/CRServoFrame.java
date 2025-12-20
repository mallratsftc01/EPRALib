package com.epra.epralib.ftclib.movement.frames;

import com.epra.epralib.ftclib.movement.Motor;
import com.qualcomm.robotcore.hardware.CRServo;

/// A [Motor] wrapper for a [CRServo].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class CRServoFrame implements Motor {

    private final CRServo servo;
    private boolean enabled;
    /// A [Motor] wrapper for a [CRServo].
    /// @param servo The servo to be wrapped
    public CRServoFrame(CRServo servo) {
        this.servo = servo;
        this.enabled = true;
    }

    /**Sets the logical direction in which this servo operates.
     * @param direction The direction to set for this servo.*/
    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction == Direction.FORWARD ? CRServo.Direction.FORWARD : CRServo.Direction.REVERSE);
    }
    /**Returns the current logical direction in which this servo is set as operating.
     * @return The current logical direction in which this servo is set as operating.*/
    @Override
    public Direction getDirection() {
        return servo.getDirection() == CRServo.Direction.FORWARD ? Direction.FORWARD : Direction.REVERSE;
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

    /// A [CRServo] can control and monitor its power.
    /// @return `True`
    @Override
    public boolean powerEnabled() { return true; }
    /// Sets the amount of power to send the motor as a float between 1 (full power forward) and -1 (full power backwards).
    ///
    /// Power is typically proportional to the rotational velocity of the motor. A power of 0 will brake the motor.
    /// @param power Power, as a double between -1 and 1
    @Override
    public void setPower(double power) { if (enabled) servo.setPower(power); }
    /// Returns the amount of power being sent to the motor as a float between 1 (full power forward) and -1 (full power backwards).
    ///
    /// Power is typically proportional to the rotational velocity of the motor. A power of 0 will brake the motor.
    /// @return The current power of this motor
    @Override
    public double getPower() { return servo.getPower(); }

    /// A [CRServo] cannot monitor its axle position.
    /// @return `False`
    @Override
    public boolean positionMonitoringEnabled() { return false; }
    /// A [CRServo] cannot control its axle position.
    /// @return `False`
    @Override
    public boolean positionControlEnabled() { return false; }

    /// Returns `NaN` as a [CRServo] cannot monitor its axle position.
    /// @return `NaN`
    @Override
    public double getCurrentPosition() { return Double.NaN; }
    /// Has no effect as a [CRServo] cannot directly control its axle position.
    /// @return `False`
    @Override
    public boolean setPosition(double position) { return false; }

    /// Returns the [CRServo] object contained within this wrapper.
    /// @return The wrapped servo
    @Override
    public Object getSelf() { return servo; }
    /// Returns the device name of the wrapped [CRServo].
    /// @return The motor's device name
    @Override
    public String toString() { return servo.getDeviceName(); }
}
