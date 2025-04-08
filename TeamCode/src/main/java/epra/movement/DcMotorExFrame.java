package epra.movement;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**A motor frame for DcMotorExs.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class DcMotorExFrame implements Motor {
    DcMotorEx motor;

    /**A motor frame for DcMotorExs.
     * @param motor A DcMotorEx.*/
    public DcMotorExFrame(DcMotorEx motor) { this.motor = motor; }

    /**Sets the logical direction in which this motor operates.
     * @param direction The direction to set for this motor.*/
    @Override
    public void setDirection(Direction direction) {
        switch (direction) {
            case FORWARD -> motor.setDirection(DcMotorSimple.Direction.FORWARD);
            case REVERSE -> motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    /**Returns the current logical direction in which this motor is set as operating.
     * @return The current logical direction in which this motor is set as operating.*/
    @Override
    public Direction getDirection() {
        return switch (motor.getDirection()) {
            case FORWARD -> Direction.FORWARD;
            case REVERSE -> Direction.REVERSE;
        };
    }

    /**Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported according to the run mode in which the motor is operating.
     Setting a power level of zero will brake the motor
     @param power The new power level of the motor, a value in the interval [-1.0, 1.0].*/
    @Override
    public void setPower(double power) { motor.setPower(power); }
    /**Returns the current configured power level of the motor.
     * @return The current level of the motor, a value in the interval [0.0, 1.0].*/
    @Override
    public double getPower() { return motor.getPower(); }

    /**Returns whether this motor is energized.*/
    @Override
    public boolean isEnabled() { return motor.isMotorEnabled(); }
    /**Individually energizes this particular motor.*/
    @Override
    public void setEnabled() { motor.setMotorEnable(); }
    /**Individually de-energizes this particular motor.*/
    @Override
    public void setDisabled() { motor.setMotorDisable(); }

    /**Returns the current reading of the encoder for this motor. The units for this reading, that is, the number of ticks per revolution, are specific to the motor/ encoder in question, and thus are not specified here.
     * @return The current reading of the encoder for this motor.*/
    @Override
    public int getCurrentPosition() { return motor.getCurrentPosition(); }

    /**Returns the DcMotorEx contained in this frame.
     * @return The DcMotorEx contained in this frame.*/
    @Override
    public Object getSelf() { return motor; }
}
