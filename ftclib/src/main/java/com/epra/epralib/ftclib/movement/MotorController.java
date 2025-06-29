package com.epra.epralib.ftclib.movement;

import androidx.annotation.NonNull;

import com.epra.epralib.ftclib.storage.MotorControllerData;
import com.epra.epralib.ftclib.storage.PIDGains;
import com.google.gson.Gson;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.time.LocalDateTime;
import java.util.Date;

/**Gives increase control over DcMotorExs.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class MotorController implements Motor {

    private Motor motor;
    private String id;

    private double velocity;
    private int savePos;
    private long saveTime;
    private long startTime;

    private PIDController pidT;
    private int startPos;
    private int targetPosition;
    private int lastTarget;
    private double lastPIDTOutput;

    private PIDController pidV;
    private double targetVelocity;
    private double lastPIDVOutput;

    private double holdPow;

    private File logJson;
    private FileWriter logWriter;
    private Gson gson;

    /**Gives increase control over DcMotorExs. Logs data in a json file on the robot for post-match analysis.
     *@param motor The motor to be used by this MotorController.
     * @param id A string that identifies the log files of this MotorController*/
    public MotorController(Motor motor, String id) throws IOException {
        this.motor = motor;
        this.id = id;

        velocity = 0;
        startPos = motor.getCurrentPosition();
        targetPosition = startPos;
        lastTarget = startPos;
        targetVelocity = 0;
        pidT = new PIDController(1, 0, 0);
        lastPIDTOutput = 0.0;
        pidV = new PIDController(1, 0, 0);
        lastPIDVOutput = 0.0;
        savePos = startPos;
        saveTime = System.currentTimeMillis();
        startTime = saveTime;
        holdPow = 0.0;

        gson = new Gson();

        SimpleDateFormat ft = new SimpleDateFormat("ddMMyyyy:HH:mm");

        logJson = AppUtil.getInstance().getSettingsFile("logs/MotorController_" + id + "_log_" + ft.format(new Date()) + ".json");
        logWriter = new FileWriter(logJson);
        logWriter.write("[");
    }

    /**Returns whether the contained Motor is energized.*/
    @Override
    public boolean isEnabled() { return motor.isEnabled(); }
    /**Energizes the Motor contained within this MotorController.*/
    @Override
    public void setEnabled() { motor.setEnabled(); }
    /**De-energizes the Motor contained within this MotorController.*/
    @Override
    public void setDisabled() { motor.setDisabled(); }

    /**Sets the logical direction in which this motor operates.
     * @param direction The direction to set for this motor.*/
    @Override
    public void setDirection(Direction direction) { motor.setDirection(direction);}
    /**Returns the current logical direction in which this motor is set as operating.
     * @return A direction, forward or reverse.*/
    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    /**Sets the motor to a certain power between -1.0 and 1.0.
     * @param power The power to set the motor to.*/
    public void setPower(double power) { motor.setPower(power + (holdPow * getCurrentPosition())); }
    /**Stops the motor.*/
    public void stop() { motor.setPower(0.0); }

    /**Saves motor data to internal logs. Also saves log data to a json file on the robot for post-match analysis.
     * @return A MotorControllerData record with data from this log.*/
    public MotorControllerData log() throws IOException {
        int posChange = motor.getCurrentPosition() - savePos;
        long timeChange = System.currentTimeMillis() - saveTime;
        savePos += posChange;
        saveTime += timeChange;
        velocity = (double) posChange / (double) timeChange;
        MotorControllerData data = new MotorControllerData(saveTime - startTime, motor.getPower(), savePos, targetPosition, velocity, targetVelocity, lastPIDTOutput, lastPIDVOutput);
        logWriter.write("\n" + gson.toJson(data) + ",");
        return data;
    }

    /**Closes the json file that this MotorController is writing to.*/
    public void closeLog() throws IOException {
        logWriter.write("]");
        logWriter.close();
    }

    /**Returns the current reading of the motor's encoder in ticks relative to the start position.
     * These ticks are specific to the encoder of a certain motor; google the ticks/revolution for your motor for best results.
     * If the encoder wire for this motor is not connected to the motor (ie. it its instead connected to an odometry pod) this number will not reflect the movement of this encoder.
     * @return The current reading of the motor's encoder. */
    public int getCurrentPosition() { return motor.getCurrentPosition() - startPos; }

    /**Returns the recent average velocity of the motor's encoder in ticks per second.
     * These ticks are specific to the encoder of a certain motor; google the ticks/revolution for your motor for best results.
     * If the encoder wire for this motor is not connected to the motor (ie. it its instead connected to an odometry pod) this number will not reflect the movement of this encoder.
     * @return The recent average velocity of the motor's encoder. */
    public double getVelocity() { return velocity; }
    /**Returns the current power being sent to the motor as a double between -1.0 and 1.0.
     * @returns The current power being sent to the robot.*/
    public double getPower() { return motor.getPower(); }

    /**Sets a target position for the motor to try to move towards.
     * @param target The position for the motor to try to move to in motor-specific ticks.*/
    public void setTarget(int target) {
        if (target != targetPosition) {
            lastTarget = targetPosition;
            targetPosition = target;
        }
    }
    /**Returns the current target position of the motor.
     * @return The target position of the motor.*/
    public int getTarget() { return targetPosition; }
    /**Checks if the position has passed through the target since the last time this method was called.
     * @param range A range around the target where the target will be considered met in motor-specific ticks.
     * @return If the position has passed through the target.*/
    public boolean checkTarget(int range) throws IOException {
        log();
        return (Math.max(getCurrentPosition(), savePos) > targetPosition && Math.min(getCurrentPosition(), savePos) < targetPosition) || Math.abs(getCurrentPosition() - targetPosition) < range;
    }
    /**Moves the motor towards the set target.
     * @param maxPower The absolute max power the motor can reach as a double between 0.0 and 1.0.
     * @param tolerance The tolerance for reaching the target as a double between 0.0 and 1.0. If this is set to 0.0 the pid will run indefinitely.
     * @param haltAtTarget If true the motor will halt once the target is reached within the set tolerance.
     * @return True once the motor reaches its target, false until then.*/
    public boolean moveToTarget(double maxPower, double tolerance, boolean haltAtTarget) {
        double p = pidT.runPID(getCurrentPosition(), targetPosition);
        lastPIDTOutput = p;
        double power = Math.min(Math.abs(p), maxPower) * Math.signum(p);
        if (Math.abs(p) > (tolerance * Math.abs(lastTarget - targetPosition))) {
            setPower(power + (holdPow * getCurrentPosition()));
            return false;
        } else {
            if (haltAtTarget) { setPower(holdPow * getCurrentPosition() ); }
            resetTargetPID();
            return true;
        }
    }

    /**Sets the hold power, a multiple added to the power to counteract gravity and hold the motor at a specific position.
     * @param holdPow A double between -1.0 and 1.0.*/
    public void setHoldPow(double holdPow) { this.holdPow = holdPow; }
    /**Returns the hold power, a multiple added to the power to counteract gravity and hold the motor at a specific position.
     * @return The hold power.*/
    public double getHoldPow() { return holdPow; }

    /**Tunes the PID loop used to reach a target.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneTargetPID(double k_p, double k_i, double k_d) { pidT.tune(k_p, k_i, k_d); }
    /**Tunes the PID loop used to reach a target.
     * @param pidGains A PIDGains record containing the gains for a PIDController.
     */
    public void tuneTargetPID(PIDGains pidGains) { pidT.tune(pidGains); }
    /**Resets the PID loop used to reach a target.*/
    public void resetTargetPID() { pidT.reset(); }

    /**Sets a target velocity for the motor to try to maintain.
     * Setting the target velocity higher than the motor's maximum velocity could lead to unwanted side effects.
     * @param target The velocity for the motor to try to maintain in motor-specific ticks/millisecond.*/
    public void setTargetVelocity(double target) { targetVelocity = target; }
    /**Maintains the targeted velocity.
     * @return True once the motor reaches its target, false until then.*/
    public boolean maintainVelocity() {
        double power = pidV.runPID(getVelocity(), targetVelocity);
        lastPIDVOutput = power;
        setPower(power);
        if (Math.abs(targetVelocity - getVelocity()) > 0.001) {
            return false;
        } else {
            return true;
        }
    }

    /**Tunes the PID loop used to maintain a velocity.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneVelocityPID(double k_p, double k_i, double k_d) { pidV.tune(k_p, k_i, k_d); }
    /**Tunes the PID loop used to maintain a velocity.
     * @param pidGains A PIDGains record containing the gains for a PIDController.*/
    public void tuneVelocityPID(PIDGains pidGains) { pidV.tune(pidGains); }
    /**Resets the PID loop used to maintain a velocity.*/
    public void resetVelocityPID() { pidV.reset(); }

    /**Sets the current position of the motor to 0.*/
    public void zero() { startPos = motor.getCurrentPosition(); }

    /**Returns the Motor used by this MotorController.
     * @return A Motor.*/
    @Override
    public Object getSelf() {
        return motor;
    }

    /**@return The id of this MotorController*/
    @NonNull
    @Override
    public String toString() { return id; }
}
