package com.epra.epralib.ftclib.movement;

import androidx.annotation.NonNull;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.storage.autonomous.MotorControllerAutoModule;
import com.epra.epralib.ftclib.storage.logdata.MotorControllerData;
import com.epra.epralib.ftclib.storage.initialization.PIDGains;
import com.google.gson.Gson;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**Gives increase control over DcMotorExs.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class MotorController implements Motor {

    private Motor motor;
    private String id;
    private int ticksPerRevolution;

    private double velocity;
    private int savePos;
    private long saveTime;
    private long startTime;

    private int startPos;
    private int targetPosition;
    private int lastTarget;
    private double lastPIDTOutput;

    private double targetVelocity;
    private double lastPIDVOutput;

    private double holdPow;

    private File logJson;
    private FileWriter logWriter;
    private Gson gson;

    /**Gives increase control over DcMotorExs. Logs data in a json file on the robot for post-match analysis.
     * Sets the initial position to the motor's current position and the ticks per revolution to 1440.
     * @param motor The motor to be used by this MotorController.
     * @param id A string that identifies the log files of this MotorController.*/
    public MotorController(Motor motor, String id) throws IOException {
        this.motor = motor;
        this.id = id;
        this.ticksPerRevolution = 1440;

        velocity = 0;
        startPos = motor.getCurrentPosition();
        targetPosition = startPos;
        lastTarget = startPos;
        targetVelocity = 0;
        PIDController.addPID(id + "_T", 1, 0, 0, this::getTargetError, false);
        PIDController.addPID(id + "_V", 1, 0, 0, this::getVelocityError, false);
        lastPIDTOutput = 0.0;
        lastPIDVOutput = 0.0;
        savePos = startPos;
        saveTime = System.currentTimeMillis();
        startTime = saveTime;
        holdPow = 0.0;

        gson = new Gson();

        SimpleDateFormat ft = new SimpleDateFormat("ddMMyyyy:HH:mm");

        logJson = AppUtil.getInstance().getSettingsFile("logs/MotorController_" + id + "_log_" + ft.format(new Date()) + ".json");
        logWriter = new FileWriter(logJson, true);
        logWriter.write("[");
    }

    /**Gives increase control over DcMotorExs. Logs data in a json file on the robot for post-match analysis.
     * Sets the ticks per revolution to 1440.
     * @param motor The motor to be used by this MotorController.
     * @param id A string that identifies the log files of this MotorController.
     * @param startPos The initial position of the motor in motor ticks.*/
    public MotorController(Motor motor, String id, int startPos) throws IOException {
        this(motor, id);
        this.startPos = startPos;
    }

    /**Gives increase control over DcMotorExs. Logs data in a json file on the robot for post-match analysis.
     * @param motor The motor to be used by this MotorController.
     * @param id A string that identifies the log files of this MotorController.
     * @param startPos The initial position of the motor in motor ticks.
     * @param ticksPerRevolution The number of motor ticks per revolution.*/
    public MotorController(Motor motor, String id, int startPos, int ticksPerRevolution) throws IOException {
        this(motor, id, startPos);
        this.ticksPerRevolution = ticksPerRevolution;
    }

    /**Gives increase control over DcMotorExs. Logs data in a json file on the robot for post-match analysis.
     * @param motor The motor to be used by this MotorController.
     * @param id A string that identifies the log files of this MotorController.
     * @param startAngle The initial angle of the motor.
     * @param ticksPerRevolution The number of motor ticks per revolution.*/
    public MotorController(Motor motor, String id, Angle startAngle, int ticksPerRevolution) throws IOException {
        this(motor, id);
        int t = motor.getCurrentPosition();
        this.startPos = (t - (t % ticksPerRevolution)) + (int) (ticksPerRevolution * (startAngle.degree() / 360));
        this.ticksPerRevolution = ticksPerRevolution;
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

    /**Returns the current angle of the motor based on the encoder reading.
     * @return The current angle of the motor.*/
    public Angle getCurrentAngle() { return Angle.degree((getCurrentPosition() % ticksPerRevolution) * 360.0 / ticksPerRevolution); }

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
        PIDController.activate(id + "_T");
        if (target != targetPosition) {
            lastTarget = targetPosition;
            targetPosition = target;
        }
    }
    /**Returns the current target position of the motor.
     * @return The target position of the motor.*/
    public int getTarget() { return targetPosition; }

    /**Returns the error between the target position and the current position.
     * @return The error to the target position.*/
    public double getTargetError() { return targetPosition - getCurrentPosition(); }
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
        double p = PIDController.get(id + "_T");
        lastPIDTOutput = p;
        double power = Math.min(Math.abs(p), maxPower) * Math.signum(p);
        if (Math.abs(p) > (tolerance * Math.abs(lastTarget - targetPosition))) {
            setPower(power + (holdPow * getCurrentPosition()));
            return false;
        } else {
            if (haltAtTarget) { setPower(holdPow * getCurrentPosition() ); }
            PIDController.idle(id + "_T");
            return true;
        }
    }

    /**Moves the motor towards the set target.
     * @param motorControllerAutoModule A MotorControllerAutoModule that contains information to run this MotorController.
     * @return True once the motor reaches its target, false until then.*/
    public boolean moveToTarget(MotorControllerAutoModule motorControllerAutoModule) {
        setTarget(motorControllerAutoModule.target());
        return moveToTarget(motorControllerAutoModule.maxPower(), motorControllerAutoModule.tolerance(), true);
    }

    /**Idles the PID loop used to reach a target position.
     * @return True if the PID loop was active, false if not.*/
    public boolean idleTargetPID() {
        return PIDController.idle(id + "_T");
    }
    /**Sets the gains of the PID loop used for reaching a target position.
     * @param pidGains The new gains for the PID loop.*/
    public void tuneTargetPID(PIDGains pidGains) {
        PIDController.tune(id + "_T", pidGains);
    }

    /**Sets the gains of the PID loop used for reaching a target position.
     * @param kp The p gain for the PID loop.
     * @param ki The i gain for the PID loop.
     * @param kd The d gain for the PID loop.*/
    public void tuneTargetPID(double kp, double ki, double kd) {
        PIDController.tune(id + "_T", kp, ki, kd);
    }

    /**Sets the hold power, a multiple added to the power to counteract gravity and hold the motor at a specific position.
     * @param holdPow A double between -1.0 and 1.0.*/
    public void setHoldPow(double holdPow) { this.holdPow = holdPow; }
    /**Returns the hold power, a multiple added to the power to counteract gravity and hold the motor at a specific position.
     * @return The hold power.*/
    public double getHoldPow() { return holdPow; }

    /**Sets a target velocity for the motor to try to maintain.
     * Setting the target velocity higher than the motor's maximum velocity could lead to unwanted side effects.
     * @param target The velocity for the motor to try to maintain in motor-specific ticks/millisecond.*/
    public void setTargetVelocity(double target) {
        PIDController.activate(id + "_V");
        targetVelocity = target;
    }

    /**Return the error between the target velocity and the current velocity.
     * @return The error of the target velocity.*/
    public double getVelocityError() { return targetVelocity - getVelocity(); }
    /**Maintains the targeted velocity.
     * @return True once the motor reaches its target, false until then.*/
    public boolean maintainVelocity() {
        double power = PIDController.get(id + "_V");
        lastPIDVOutput = power;
        setPower(power);
        if (Math.abs(targetVelocity - getVelocity()) > 0.001) {
            return false;
        } else {
            return true;
        }
    }

    /**Idles the PID loop used to maintain velocity.
     * @return True if the PID loop was active, false if not.*/
    public boolean idleVelocityPID() {
        return PIDController.idle(id + "_V");
    }

    /**Sets the gains of the PID loop used for maintaining velocity.
     * @param pidGains The new gains for the PID loop.*/
    public void tuneVelocityPID(PIDGains pidGains) {
        PIDController.tune(id + "_V", pidGains);
    }

    /**Sets the gains of the PID loop used for maintaing velocity.
     * @param kp The p gain for the PID loop.
     * @param ki The i gain for the PID loop.
     * @param kd The d gain for the PID loop.*/
    public void tuneVelocityPID(double kp, double ki, double kd) {
        PIDController.tune(id + "_V", kp, ki, kd);
    }

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
