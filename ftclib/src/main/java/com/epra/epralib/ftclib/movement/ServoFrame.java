package com.epra.epralib.ftclib.movement;

import com.qualcomm.robotcore.hardware.Servo;

/**A motor frame for Servos.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class ServoFrame implements Motor{

    private final Servo servo;
    private boolean enabled;
    private double precision;

    /**A motor frame for Servos.
     * @param servo A Servo.
     * @param precision The number of digits to be returned by the <code>getCurrentPosition</code> method.
     * (i.e., for a position of 0.31415, precision of 1 would return '3' whereas precision of 4 would return `3142`.)*/
    public ServoFrame(Servo servo, int precision) {
        this.servo = servo;
        this.enabled = true;
        this.precision = Math.pow(10, precision);
    }

    /**A motor frame for Servos with the precision set to 5.
     * @param servo A Servo.*/
    public ServoFrame(Servo servo) {
        this(servo, 5);
    }

    /**Sets the logical direction in which this servo operates.
     * @param direction The direction to set for this servo.*/
    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction == Direction.FORWARD ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
    }
    /**Returns the current logical direction in which this servo is set as operating.
     * @return The current logical direction in which this servo is set as operating.*/
    @Override
    public Direction getDirection() {
        return (servo.getDirection() == Servo.Direction.FORWARD) ? Direction.FORWARD : Direction.REVERSE;
    }

    /**A Servo cannot be directly set to run at a specific power.
     * @return False*/
    public boolean powerEnabled() { return false; }
    /**As a Servo cannot be set to run at a specific power, this method does nothing.*/
    @Override
    public void setPower(double power) { }
    /**Will return NaN to indicate that this is a Servo that cannot be set to run at a specific power.
     * @return NaN*/
    @Override
    public double getPower() {
        return Double.NaN;
    }

    /**Returns whether this servo is energized.*/
    @Override
    public boolean isEnabled() { return enabled; }
    /**Individually energizes this particular servo.*/
    @Override
    public void setEnabled() { enabled = true; }
    /**Individually de-energizes this particular servo.*/
    @Override
    public void setDisabled() { enabled = false; }

    /**Sets the precision for this servo.
     * (i.e., for a position of 0.31415, precision of 1 would return '3' whereas precision of 4 would return `3142`.)
     * @param precision The number of digits to be returned by the <code>getCurrentPosition</code> method.*/
    public void setPrecision(int precision) { this.precision = Math.pow(10, precision); }
    /**Returns the precision of this servo.
     * (i.e., for a position of 0.31415, precision of 1 would return '3' whereas precision of 4 would return `3142`.)*/
    public int getPrecision() { return (int) Math.round(Math.log10(precision)); }

    /**Servos can read their current position.
     * @return True*/
    @Override
    public boolean positionEnabled() { return true; }
    /**Returns the current position of the servo with the precision set on initialization or through
     * the setPrecision method.
     * (i.e., for a position of 0.31415, precision of 1 would return '3' whereas precision of 4 would return `3142`.)
     * @return The current position of the servo.*/
    @Override
    public int getCurrentPosition() {
        return (int) Math.round(servo.getPosition() * precision);
    }
    /**Returns the current position of the servo as a double between 0 and 1, not using the precision scheme.
     * @return The current position reading of the servo.*/
    public double getCurrentPositionDouble() {
        return servo.getPosition();
    }

    /**Moves the servo to a position specified in the same precision scheme as used by <code>getCurrentPosition</code>.
     * @param position The position for the servo to move to.
     * @return True, as long as the servo is enabled.*/
    @Override
    public boolean setPosition(int position) {
        if (!enabled) return false;
        servo.setPosition(position / precision);
        return true;
    }
    /**Moves the servo to a position specified by a double between 0 and 1, not in the precision scheme.
     * @param position The position for the servo to move to, between 0 and 1.*/
    public void setPosition(double position) { if (enabled) servo.setPosition(position); }

    /**Returns the Servo contained in this frame.
     * @return The Servo contained in this frame.*/
    @Override
    public Object getSelf() {
        return servo;
    }
}
