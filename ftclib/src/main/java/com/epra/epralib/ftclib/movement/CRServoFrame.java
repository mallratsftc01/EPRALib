package com.epra.epralib.ftclib.movement;

import com.qualcomm.robotcore.hardware.CRServo;

/**A motor frame for CRServos.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class CRServoFrame implements Motor {

    private final CRServo servo;
    private boolean enabled;
    /**A motor frame for CRServos.
     * @param servo A CRServo.*/
    public CRServoFrame(CRServo servo) {
        this.servo = servo;
        this.enabled = true;
    }

    /**The power can be set for CRServos.
     * @return True*/
    @Override
    public boolean powerEnabled() { return true; }

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

    /**Sets the power level of the servo, expressed as a fraction of the maximum possible power / speed supported according to the run mode in which the servo is operating.
     Setting a power level of zero will brake the servo.
     @param power The new power level of the servo, a value between -1 and 1.*/
    @Override
    public void setPower(double power) { if (enabled) servo.setPower(power); }
    /**Returns the current configured power level of the servo.
     * @return The current power level of the servo, a value between -1 and 1.*/
    @Override
    public double getPower() { return servo.getPower(); }

    /**Returns whether this servo is energized.*/
    @Override
    public boolean isEnabled() { return enabled; }
    /**Individually energizes this particular servo.*/
    @Override
    public void setEnabled() { enabled = true; }
    /**Individually de-energizes this particular servo.*/
    @Override
    public void setDisabled() { enabled = false; }

    /**CRServos cannot read their current position.
     * @return False*/
    @Override
    public boolean positionEnabled() { return false; }

    /**Returns 0 as CRServos cannot read their current positions.
     * @return 0*/
    @Override
    public int getCurrentPosition() { return 0; }
    /**CRServos cannot directly set their positions.
     * @return False*/
    @Override
    public boolean setPosition(int position) { return false; }

    /**Returns the CRServo contained in this frame.
     * @return The CRServo contained in this frame.*/
    @Override
    public Object getSelf() { return servo; }
}
