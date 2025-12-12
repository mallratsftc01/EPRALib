package com.epra.epralib.ftclib.movement;

import androidx.annotation.NonNull;

import com.epra.epralib.ftclib.storage.logdata.DataLogger;
import com.qualcomm.robotcore.hardware.Servo;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.storage.autonomous.MotorControllerAutoModule;
import com.epra.epralib.ftclib.storage.initialization.PIDGains;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/// Wraps a [Motor] for increased control and flexibility.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class MotorController implements Motor, DataLogger {

    /// Data that the motor controller can log, if requested.
    public enum LogTarget {
        POWER, POSITION, TARGET_POSITION, POSITION_PID, VELOCITY, TARGET_VELOCITY, VELOCITY_PID, ACCELERATION
    }

    private final Motor motor;

    /// A tag that will identify the log files from this motor controller.
    ///
    /// Log files can be found in the robot controller's internal file system at
    /// `sdcard/FIRST/settings/logs/MotorController_ID_log_DATETIME.json`.
    ///
    /// **Will not log if [#positionMonitoringEnabled] is `False`.**
    private final String id;
    /// The number of ticks per one rotation for this motor controller.
    ///
    /// The number of ticks in one revolution varies by motor, but should be listed on the motor's product page.
    ///
    /// - A [REV HD Hex Motor](https://www.revrobotics.com/REV-41-1291/) with no gearboxes will have 28 ticks per revolution.
    /// - A [REV Core Hex Motor](https://www.revrobotics.com/rev-41-1300/) will have 288 ticks per revolution.
    /// - The ticks per revolution for a [Servo] will depend on where the right and left positions are set.
    private final double ticksPerRevolution;

    private double velocity;
    private double acceleration;
    private double savePos;
    private long saveTime;
    private final long startTime;

    /// The start position for the motor controller.
    ///
    /// This position is seen as 0 by the motor controller.
    /// The outputs of [#getCurrentPosition] will be relative to the start position.
    /// The units of this position are specific to the motor.
    private double startPos;
    private double targetPosition;
    private double lastTarget;
    private double lastPIDTOutput;

    private double targetVelocity;
    private double lastPIDVOutput;

    private double holdPow;

    private final DriveTrain.Orientation driveOrientation;

    private final String logPath;
    private final LogTarget[] loggingTargets;
    private final HashMap<String, Double> logData;

    /// Wraps a [Motor] for increased control and flexibility.
    ///
    /// Logs JSON files on the robot for post-match analysis.
    ///
    /// PID loops are not initialized. They can be initialized through the tune functions.
    /// @param motor The motor to be wrapped
    /// @param id A tag that will identify the log files from this motor controller
    /// @param startPosition The start position for the motor controller
    /// @param ticksPerRevolution The number of ticks in one revolution of the motor's axle
    /// @param direction The direction for this motor to operate in
    /// @param driveOrientation The drive orientation of this motor if it is a drive motor (can be null if this motor is not a drive motor)
    /// @param logTargets An array of the data types that this motor controller should log
    ///
    /// @see #id
    /// @see #startPos
    /// @see #ticksPerRevolution
    /// @see com.epra.epralib.ftclib.movement.DriveTrain.Orientation
    public MotorController(Motor motor, String id, double startPosition, double ticksPerRevolution, Direction direction, DriveTrain.Orientation driveOrientation, LogTarget[] logTargets) {
        this.motor = motor;
        this.id = id;
        this.ticksPerRevolution = ticksPerRevolution;
        this.motor.setDirection(direction);
        this.driveOrientation = driveOrientation;
        this.loggingTargets = logTargets;

        velocity = 0;
        startPos = startPosition;
        targetPosition = startPos;
        lastTarget = startPos;
        targetVelocity = 0;
        lastPIDTOutput = 0.0;
        lastPIDVOutput = 0.0;
        savePos = startPos;
        saveTime = System.currentTimeMillis();
        startTime = saveTime;
        holdPow = 0.0;

        logData = new HashMap<>();
        logPath = "MotorController_" + id + ".json";
    }

    /// Wraps a [Motor] for increased control and flexibility.
    ///
    /// Logs JSON files on the robot for post-match analysis.
    ///
    /// Default parameters:
    /// - The id is the device name of the motor.
    /// - The starting position is the current position of the motor, or 0 if [#positionMonitoringEnabled] is `False` for the motor.
    /// - The ticks per revolution are 28, default for a [REV HD Hex Motor](https://www.revrobotics.com/REV-41-1291/) with no gearboxes.
    /// - PID loops not initialized. They can be initialized through the tune functions.
    /// - [Direction#FORWARD].
    /// - No [DriveTrain.Orientation].
    /// - No [LogTarget]s.
    /// @param motor The motor to be wrapped
    public MotorController(Motor motor) { this(motor, motor.toString(), motor.getCurrentPosition(), 28, Direction.FORWARD, null, new LogTarget[] {}); }
    /// Wraps a [Motor] for increased control and flexibility.
    ///
    /// Logs JSON files on the robot for post-match analysis.
    ///
    /// Default parameters:
    /// - The starting position is the current position of the motor, or 0 if [#positionMonitoringEnabled] is `False` for the motor.
    /// - The ticks per revolution are 28, default for a [REV HD Hex Motor](https://www.revrobotics.com/REV-41-1291/) with no gearboxes.
    /// - PID loops not initialized. They can be initialized through the tune functions.
    /// - [Direction#FORWARD].
    /// - No [DriveTrain.Orientation].
    /// - No [LogTarget]s.
    /// @param motor The motor to be wrapped
    /// @param id A tag that will identify the log files from this motor controller
    ///
    /// @see #id
    public MotorController(Motor motor, String id) { this(motor, id, motor.getCurrentPosition(), 28, Direction.FORWARD, null, new LogTarget[] {}); }

    /// A builder for a [MotorController].
    public static class Builder {
        private final Motor motor;
        private String id;
        private double startPosition;
        private double ticksPerRevolution;
        private DriveTrain.Orientation driveOrientation;
        private Direction direction;
        private double kp_t, ki_t, kd_t;
        private double kp_v, ki_v, kd_v;
        private boolean tuneT, tuneV;
        private final ArrayList<LogTarget> loggingTargets;
        /// A builder for a [MotorController].
        ///
        /// Starts with default parameters:
        /// - The id is the device name of the motor.
        /// - The starting position is the current position of the motor, or 0 if [#positionMonitoringEnabled] is `False` for the motor.
        /// - The ticks per revolution are 28, default for a [REV HD Hex Motor](https://www.revrobotics.com/REV-41-1291/) with no gearboxes.
        /// - PID loops not initialized.
        /// - [.Direction#FORWARD].
        /// - No [DriveTrain.Orientation].
        /// - No [LogTarget]s.
        /// @param motor The motor to be wrapped
        ///
        /// @see #id(String)
        /// @see #startPosition(double)
        /// @see #startAngle(Angle)
        /// @see #ticksPerRevolution(double)
        /// @see #driveOrientation(DriveTrain.Orientation)
        /// @see #targetPIDConstants(double, double, double)
        /// @see #velocityPIDConstants(double, double, double)
        /// @see #addLogTarget(LogTarget...) 
        /// @see #build()
        public Builder (Motor motor) {
            this.motor = motor;
            id = motor.toString();
            startPosition = (motor.positionMonitoringEnabled()) ? motor.getCurrentPosition() : 0;
            ticksPerRevolution = 28;
            direction = Direction.FORWARD;
            tuneT = tuneV = false;
            loggingTargets = new ArrayList<>();
        }
        /// Sets the id for the [MotorController].
        ///
        /// Log files can be found in the robot controller's internal file system at
        /// `sdcard/FIRST/settings/logs/MotorController_ID_log_DATETIME.json`.
        ///
        /// **Will not log if [#positionMonitoringEnabled] is `False`.**
        /// @param id A tag that will identify the log files from this motor controller
        /// @return This builder
        public Builder id(String id) { this.id = id; return this; }
        /// Sets the start position for the [MotorController].
        ///
        /// This position is seen as 0 by the motor controller.
        /// The outputs of [#getCurrentPosition] will be relative to the start position.
        /// The units of this position are specific to the motor.
        /// @param startPosition The start position for the motor controller
        /// @return This builder
        ///
        /// @see #ticksPerRevolution(double)
        /// @see #startAngle(Angle)
        /// @see #startPositionOffset(double)
        /// @see #startAngleOffset(Angle)
        public Builder startPosition(double startPosition) { this.startPosition = startPosition; return this; }
        /// Sets the start position for the [MotorController] by offsetting from the current position.
        ///
        /// This position is seen as 0 by the motor controller.
        /// The outputs of [#getCurrentPosition] will be relative to the start position.
        /// The units of this position are specific to the motor.
        /// If [#positionMonitoringEnabled] is `False`, this will have no effect.
        /// @param offset The offset to add to the current position
        /// @return This builder
        ///
        /// @see #ticksPerRevolution(double)
        /// @see #startPosition(double)
        /// @see #startPositionOffset(double)
        /// @see #startAngleOffset(Angle)
        public Builder startPositionOffset(double offset) { if (motor.positionMonitoringEnabled()) this.startPosition = offset + motor.getCurrentPosition(); return this; }
        /// Sets the start [Angle] for the [MotorController].
        ///
        /// Calculates a start position based on the start angle and the ticks per revolution.
        ///
        /// This start position is seen as 0 by the motor controller.
        /// The outputs of [#getCurrentPosition] will be relative to the start position.
        ///
        /// If [#positionMonitoringEnabled] is `True` for the motor, the start position will be offset to make the start
        /// angle relative to the 0 degrees clockwise from the current position of the motor axle.
        /// @param startAngle The angle from which the start position will be calculated
        /// @return This builder
        ///
        /// @see #ticksPerRevolution(double)
        /// @see #startPosition(double)
        /// @see #startAngle(Angle)
        /// @see #startAngleOffset(Angle)
        public Builder startAngle(Angle startAngle) {
            this.startPosition = startAngle.radian() * ticksPerRevolution / (2 * Math.PI);
            if (motor.positionMonitoringEnabled()) { this.startPosition += motor.getCurrentPosition() - (motor.getCurrentPosition() % ticksPerRevolution); }
            return this;
        }
        /// Sets the start position for the [MotorController] by offsetting by an [Angle] from the current position.
        ///
        /// This position is seen as 0 by the motor controller.
        /// The outputs of [#getCurrentPosition] will be relative to the start position.
        /// The units of this position are specific to the motor.
        /// If [#positionMonitoringEnabled] is `False`, this will have no effect.
        /// @param offset The angular offset to add to the current position
        /// @return This builder
        ///
        /// @see #ticksPerRevolution(double)
        /// @see #startPosition(double)
        /// @see #startAngle(Angle)
        /// @see #startPositionOffset(double)
        /// @see #targetPIDConstants(double, double, double)
        /// @see #velocityPIDConstants(double, double, double)
        public Builder startAngleOffset(Angle offset) { this.startPositionOffset(offset.radian() * ticksPerRevolution / (2 * Math.PI)); return this;}
        /// Sets the ticks per rotation for the [MotorController].
        ///
        /// The number of ticks in one revolution varies by motor, but should be listed on the motor's product page.
        ///
        /// - A [REV HD Hex Motor](https://www.revrobotics.com/REV-41-1291/) with no gearboxes will have 28 ticks per revolution.
        /// - A [REV Core Hex Motor](https://www.revrobotics.com/rev-41-1300/) will have 288 ticks per revolution.
        /// - The ticks per revolution for a [Servo] will depend on where the right and left positions are set.
        ///
        /// @param ticksPerRevolution The number of ticks in one revolution of the motor's axle
        /// @return This builder
        public Builder ticksPerRevolution(double ticksPerRevolution) { this.ticksPerRevolution = ticksPerRevolution; return this; }
        /// Sets the [Direction] for the motor to operate in.
        /// @param direction The direction for the motor to operate in
        /// @return This builder
        public Builder direction(Direction direction) { this.direction = direction; return this; }
        /// Sets an [DriveTrain.Orientation] for this motor controller.
        ///
        /// This allows for the [DriveTrain] to automatically recognize this motor controller's orientation without needing
        /// to set it separately. Only use if this motor is a drive motor.
        /// @param orientation The orientation of this motor on the robot
        /// @see DriveTrain.Builder#motor(MotorController)
        public Builder driveOrientation(DriveTrain.Orientation orientation) { this.driveOrientation = orientation; return this; }
        /// Tunes and initializes the PID loop used in [#moveToTarget(double, double, boolean)] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param kp The `p` constant
        /// @param ki The `i` constant
        /// @param kd The `d` constant
        /// @return This builder
        ///
        /// @see PIDController
        public Builder targetPIDConstants(double kp, double ki, double kd) {
            kp_t = kp; ki_t = ki; kd_t = kd;
            tuneT = true;
            return this;
        }
        /// Tunes and initializes the PID loop used in [#moveToTarget(double, double, boolean)] using
        /// [PIDController#tune(String, PIDGains)].
        /// @param pidGains A record with instructions to tune the PID loop
        /// @return This builder
        ///
        /// @see PIDController
        public Builder targetPIDConstants(PIDGains pidGains) {
            kp_t = pidGains.kp(); ki_t = pidGains.ki(); kd_t = pidGains.kd();
            tuneT = true;
            return this;
        }
        /// Tunes and initializes the PID loop used in [#maintainVelocity] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param kp The `p` constant
        /// @param ki The `i` constant
        /// @param kd The `d` constant
        /// @return This builder
        ///
        /// @see PIDController
        public Builder velocityPIDConstants(double kp, double ki, double kd) {
            kp_v = kp; ki_v = ki; kd_v = kd;
            tuneV = true;
            return this;
        }
        /// Tunes and initializes the PID loop used in [#maintainVelocity] using
        /// [PIDController#tune(String, PIDGains)].
        /// @param pidGains A record with instructions to tune the PID loop
        /// @return This builder
        ///
        /// @see PIDController
        public Builder velocityPIDConstants(PIDGains pidGains) {
            kp_v = pidGains.kp(); ki_v = pidGains.ki(); kd_v = pidGains.kd();
            tuneV = true;
            return this;
        }
        
        /// Adds any number of [LogTarget]s to the [MotorController].
        /// @param logTargets Any number of data types that the motor controller should log
        /// @return This builder
        public Builder addLogTarget(LogTarget... logTargets) {
            this.loggingTargets.addAll(Arrays.asList(logTargets));
            return this;
        }

        /// Builds a [MotorController] from this builder.
        /// @return The motor controller built by this builder
        public MotorController build() {
            MotorController mc = new MotorController(motor, id, startPosition, ticksPerRevolution, direction, driveOrientation, loggingTargets.toArray(new LogTarget[0]));
            if (tuneT) mc.tuneTargetPID(kp_t, ki_t, kd_t);
            if (tuneV) mc.tuneVelocityPID(kp_v, ki_v, kd_v);
            return mc;
        }
    }

    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public boolean isEnabled() { return motor.isEnabled(); }
    /// {@inheritDoc}
    @Override
    public void setEnabled() { motor.setEnabled(); }
    /// {@inheritDoc}
    @Override
    public void setDisabled() { motor.setDisabled(); }

    /// {@inheritDoc}
    /// @param direction {@inheritDoc}
    @Override
    public void setDirection(Direction direction) { motor.setDirection(direction);}
    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    /// Returns the [DriveTrain.Orientation] of this motor if it is drive motor, or null if it is not.
    /// @return The orientation of this motor
    public DriveTrain.Orientation getDriveOrientation() { return this.driveOrientation; }

    /// {@inheritDoc}
    /// @return If the amount of power sent to the wrapped motor can be controlled or monitored
    @Override
    public boolean powerEnabled() { return motor.powerEnabled(); }
    /// {@inheritDoc}
    /// @param power {@inheritDoc}
    public void setPower(double power) { motor.setPower(power + ((positionMonitoringEnabled()) ? (holdPow * getCurrentPosition()) : 0)); }
    /// Stops the wrapped motor by setting its power to 0.
    /// @see #setPower(double)
    public void stop() { motor.setPower(0.0); }

    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public double getPower() { return motor.getPower(); }

    /// {@inheritDoc}
    /// @return If the position of the wrapped motor's axle can be monitored
    @Override
    public boolean positionMonitoringEnabled() { return motor.positionMonitoringEnabled(); }

    /// {@inheritDoc}
    /// @return If the position of the wrapped motor's axle can be controlled
    @Override
    public boolean positionControlEnabled() { return motor.positionControlEnabled(); }
    /// Returns the current rotational position of the motor's axle relative to the start position.
    ///
    /// The units of this position are specific to the motor.
    /// If [#positionMonitoringEnabled] is `False`, this will instead return `NaN`.
    /// @return The current position of this motor
    ///
    /// @see #startPos
    @Override
    public double getCurrentPosition() { return (positionMonitoringEnabled()) ? motor.getCurrentPosition() - startPos : Double.NaN; }

    /// Returns the current angle of the motor's axle based on the reading from [#getCurrentPosition].
    ///
    /// This calculation is based on the ticks per revolution of this motor.
    /// If [#positionMonitoringEnabled] is `False`, this will instead return `NaN`.
    /// @return The current angle of the motor's axle
    ///
    /// @see #ticksPerRevolution
    public Angle getCurrentAngle() { return (positionMonitoringEnabled()) ? Angle.degree((getCurrentPosition() % ticksPerRevolution) * 360.0 / ticksPerRevolution) : null; }

    /// Returns the recent average rotational velocity of the motor's axle.
    ///
    /// The velocity is in units of ticks per second.
    ///
    /// The velocity is updated by calling [#updateLog]. If [#updateLog] is not called frequently, the velocity value will be inaccurate.
    /// If [#positionMonitoringEnabled] is `False`, this will instead return `NaN`.
    /// @return The recent average velocity of the motor
    ///
    /// @see #ticksPerRevolution
    /// @see #getRPS()
    /// @see #getRPM()
    public double getVelocity() { return (positionMonitoringEnabled()) ? velocity : Double.NaN; }

    /// Returns the recent average rotational velocity of the motor's axle.
    ///
    /// The velocity is in units of revolutions per second, found using the ticks per revolution.
    ///
    /// The velocity is updated by calling [#updateLog]. If [#updateLog] is not called frequently, the velocity value will be inaccurate.
    /// If [#positionMonitoringEnabled] is `False`, this will instead return `NaN`.
    /// @return The recent average velocity of the motor
    ///
    /// @see #ticksPerRevolution
    /// @see #getVelocity()
    /// @see #getRPM()
    public double getRPS() { return (positionMonitoringEnabled()) ? velocity / ticksPerRevolution : Double.NaN;}

    /// Returns the recent average rotational velocity of the motor's axle.
    ///
    /// The velocity is in units of revolutions per minute, found using the ticks per revolution.
    ///
    /// The velocity is updated by calling [#updateLog]. If [#updateLog] is not called frequently, the velocity value will be inaccurate.
    /// If [#positionMonitoringEnabled] is `False`, this will instead return `NaN`.
    /// @return The recent average velocity of the motor
    ///
    /// @see #ticksPerRevolution
    /// @see #getVelocity()
    /// @see #getRPS()
    public double getRPM() { return (positionMonitoringEnabled()) ? velocity * 60 / ticksPerRevolution : Double.NaN;}

    /// Sets a target position for the motor controller in ticks.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will have no effect.
    /// @param target A target position for the motor controller
    ///
    /// @see #ticksPerRevolution
    /// @see #setTargetVelocity(double)
    public void setTarget(double target) {
        if (positionMonitoringEnabled()) {
            if (powerEnabled()) {
                PIDController.activate(id + "_T");
            }
            if (target != targetPosition) {
                lastTarget = targetPosition;
                targetPosition = target;
            }
        }
    }
    /// Returns the current target position of the motor controller in ticks.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will return `NaN` instead.
    /// @return The target position of the motor controller
    ///
    /// @see #ticksPerRevolution
    /// @see #getTargetVelocity()
    public double getTarget() { return (positionMonitoringEnabled()) ? targetPosition : Double.NaN; }

    /// Returns the absolute value of the error between the target position and the current position in ticks.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will return `NaN` instead.
    /// @return The error from the target
    ///
    /// @see #ticksPerRevolution
    /// @see #setTarget(double)
    /// @see #getTarget()
    public double getTargetError() { return (positionMonitoringEnabled()) ? Math.abs(targetPosition - getCurrentPosition()) : Double.NaN; }
    /// Returns if the motor has rotated through a range around the target position since the last time this function was called.
    ///
    /// Calls [#updateLog].
    /// If [#positionMonitoringEnabled] is `False`, this will return `False` instead.
    /// @param range A range around the target position
    /// @return If the motor has rotated through the target position since the last call
    ///
    /// @see #ticksPerRevolution
    /// @see #setTarget(double)
    /// @see #getTarget()
    public boolean checkTarget(double range) {
        if (!positionMonitoringEnabled()) { return false; }
        updateLog();
        return (Math.max(getCurrentPosition(), savePos) > targetPosition && Math.min(getCurrentPosition(), savePos) < targetPosition) || Math.abs(getCurrentPosition() - targetPosition) < range;
    }

    /// {@inheritDoc} [#moveToTarget(double, double, boolean)] may be a good alternative.
    ///
    /// @see #ticksPerRevolution
    @Override
    public boolean setPosition(double position) { return motor.setPosition(position); }
    /// Rotates the motor to the target position, returns if that target position has been reached.
    ///
    /// Will use [#setPosition(double)] if [#positionControlEnabled] is `True`.
    /// Uses PID loops from [PIDController] otherwise.
    /// PID loops must be updated frequently with [PIDController#update] or this function will be ineffective.
    ///
    /// The tolerance controls how precise the motor must be in reaching the target.
    /// A tolerance of 0 or less will result in the motor never reaching the target.
    /// A tolerance of 1 or more will result in the motor never moving.
    ///
    /// If [#positionControlEnabled] is `False` and either [#powerEnabled] or [#positionMonitoringEnabled]
    /// is `False`, this will have no effect and return `False`.
    /// @param maxPower The absolute maximum power that can be used to reach the target position
    /// @param tolerance The tolerance for positioning as a number between 0 and 1
    /// @param haltAtTarget If the motor's power is cut once it reaches the target position
    /// @return If the motor has reached the target
    ///
    /// @see #setTarget(double)
    /// @see #getTarget()
    /// @see #getCurrentPosition()
    public boolean moveToTarget(double maxPower, double tolerance, boolean haltAtTarget) {
        if (positionControlEnabled()) {
            return setPosition(targetPosition);
        } else if (powerEnabled() && positionMonitoringEnabled()) {
            double p = PIDController.get(id + "_T");
            lastPIDTOutput = p;
            double power = Math.min(Math.abs(p), maxPower) * Math.signum(p);
            if (Math.abs(p) >= (tolerance * Math.abs(lastTarget - targetPosition))) {
                setPower(power + (holdPow * getCurrentPosition()));
                return false;
            } else {
                if (haltAtTarget) {
                    setPower(holdPow * getCurrentPosition());
                }
                PIDController.idle(id + "_T");
                return true;
            }
        } else {
            return false;
        }
    }

    /// Rotates the motor to the target position using a [MotorControllerAutoModule],
    /// returns if that target position has been reached.
    ///
    /// Will use [#setPosition(double)] if [#positionControlEnabled] is `True`.
    /// Uses PID loops from [PIDController] otherwise.
    /// PID loops must be updated frequently with [PIDController#update] or this function will be ineffective.
    ///
    /// The tolerance controls how precise the motor must be in reaching the target.
    /// A tolerance of 0 or less will result in the motor never reaching the target.
    /// A tolerance of 1 or more will result in the motor never moving.
    ///
    /// If [#positionControlEnabled] is `False` and either [#powerEnabled] or [#positionMonitoringEnabled]
    /// is `False`, this will have no effect and return `False`.
    /// @param motorControllerAutoModule An auto module with instructions to rotate the motor to a target
    /// @return If the motor has reached the target
    ///
    /// @see #setTarget(double)
    /// @see #getTarget()
    /// @see #getCurrentPosition()
    public boolean moveToTarget(MotorControllerAutoModule motorControllerAutoModule) {
        setTarget(motorControllerAutoModule.target());
        return moveToTarget(motorControllerAutoModule.maxPower(), motorControllerAutoModule.tolerance(), true);
    }

    /// Idles the PID loop used in [#moveToTarget(double, double, boolean)] using [PIDController#idle(String)].
    /// @return If the PID loop was active before it was idled
    /// @see PIDController
    public boolean idleTargetPID() {
        return PIDController.idle(id + "_T");
    }
    /// Tunes the PID loop used in [#moveToTarget(double, double, boolean)] using
    /// [PIDController#tune(String, PIDGains)].
    /// @param pidGains A record with instructions to tune the PID loop
    ///
    /// @see PIDController
    public void tuneTargetPID(PIDGains pidGains) {
        if (PIDController.hasPID(id + "_T")) {
            PIDController.tune(id + "_T", pidGains);
        } else {
            PIDController.addPID(id + "_T", pidGains, this::getTargetError, false);
        }
    }

    /// Tunes the PID loop used in [#moveToTarget(double, double, boolean)] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    ///
    /// @see PIDController
    public void tuneTargetPID(double kp, double ki, double kd) {
        if (PIDController.hasPID(id + "_T")) {
            PIDController.tune(id + "_T", kp, ki, kd);
        } else {
            PIDController.addPID(id + "_T", kp, ki, kd, this::getTargetError, false);
        }
    }

    /// Sets the hold power, a multiple added to the power to counteract gravity and hold the motor at a specific position.
    ///
    /// The hold power should be between -1 and 1.
    /// If either [#powerEnabled] or [#positionMonitoringEnabled] is `False`, hold power will have no effect.
    /// @param holdPow The hold power
    ///
    /// @see #setPower(double)
    /// @see #getHoldPow()
    public void setHoldPow(double holdPow) { this.holdPow = holdPow; }
    /// Returns the hold power, a multiple added to the power to counteract gravity and hold the motor at a specific position.
    /// @return The hold power
    ///
    /// @see #setPower(double)
    /// @see #setHoldPow(double)
    public double getHoldPow() { return holdPow; }

    /// Sets a target rotational velocity for the motor controller to maintain in ticks per second.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will have no effect.
    /// @param target A target position for the motor controller
    ///
    /// @see #ticksPerRevolution
    /// @see #getVelocity()
    /// @see #setTarget(double)
    public void setTargetVelocity(double target) {
        if (positionMonitoringEnabled()) {
            PIDController.activate(id + "_V");
            targetVelocity = target;
        }
    }
    /// Returns the current target rotational velocity of the motor controller in ticks per second.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will return `NaN` instead.
    /// @return The target position of the motor controller
    ///
    /// @see #ticksPerRevolution
    /// @see #getVelocity()
    /// @see #getTarget()
    public double getTargetVelocity() { return (positionMonitoringEnabled()) ? targetVelocity : Double.NaN; }

    /// Returns the absolute value of the error between the target rotational velocity and the current rotational velocity in ticks per second.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will return `NaN` instead.
    /// @return The error from the target
    ///
    /// @see #ticksPerRevolution
    /// @see #getVelocity()
    /// @see #setTargetVelocity(double)
    /// @see #getTargetVelocity()
    public double getVelocityError() { return (positionMonitoringEnabled()) ? Math.abs(targetVelocity - getVelocity()) : Double.NaN; }
    /// Maintains the targeted rotational velocity.
    ///
    /// Uses PID loops from [PIDController].
    /// PID loops must be updated frequently with [PIDController#update] or this function will be ineffective.
    ///
    /// If either [#powerEnabled] or [#positionMonitoringEnabled]
    /// is `False`, this will have no effect and return `False`.
    /// @return If the motor is at the target velocity
    ///
    /// @see #getVelocity()
    /// @see #setTargetVelocity(double)
    /// @see #getTargetVelocity()
    public boolean maintainVelocity() {
        if (!powerEnabled() || !positionMonitoringEnabled()) { return false; }
        double power = PIDController.get(id + "_V");
        lastPIDVOutput = power;
        setPower(power);
        return !(Math.abs(targetVelocity - getVelocity()) > 0.001);
    }

    /// Idles the PID loop used in [#maintainVelocity] using [PIDController#idle(String)].
    /// @return If the PID loop was active before it was idled
    /// @see PIDController
    public boolean idleVelocityPID() {
        return PIDController.idle(id + "_V");
    }

    /// Tunes the PID loop used in [#maintainVelocity] using
    /// [PIDController#tune(String, PIDGains)].
    /// @param pidGains A record with instructions to tune the PID loop
    ///
    /// @see PIDController
    public void tuneVelocityPID(PIDGains pidGains) {
        if (PIDController.hasPID(id + "_V")) {
            PIDController.tune(id + "_V", pidGains);
        } else {
            PIDController.addPID(id + "_V", pidGains, this::getVelocityError, false);
        }
    }

    /// Tunes the PID loop used in [#maintainVelocity] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    ///
    /// @see PIDController
    public void tuneVelocityPID(double kp, double ki, double kd) {
        if (PIDController.hasPID(id + "_V")) {
            PIDController.tune(id + "_V", kp, ki, kd);
        } else {
            PIDController.addPID(id + "_V", kp, ki, kd, this::getVelocityError, false);
        }
    }

    /// Changes the start position to the current position.
    ///
    /// If [#positionMonitoringEnabled] is `False`, this will have no effect.
    ///
    /// @see #startPos
    /// @see #getCurrentPosition()
    /// @see #ticksPerRevolution
    public void zero() { if (positionMonitoringEnabled()) startPos = motor.getCurrentPosition(); }

    /// {@inheritDoc}
    /// @return The relative file path for the logs from this logger
    @Override
    public String getLogPath() { return logPath; }
    /// {@inheritDoc}
    /// @return A hash map with a data snapshot of this logger
    @Override
    public HashMap<String, Double> logData() { return logData; }
    /// Updates velocity and acceleration and saves data to an internal log that can be accessed through [#logData].
    ///
    /// If [#powerEnabled] is `False`, power data cannot be logged. 
    /// If [#positionMonitoringEnabled] is `False`, position-based data cannot be logged.
    /// If both are false, no data will be logged and this will return `False`.
    /// @return If log values were successfully updated
    @Override
    public boolean updateLog() {
        if (powerEnabled() && Arrays.asList(loggingTargets).contains(LogTarget.POWER)) { logData.put("power", motor.getPower()); }
        if (!this.positionMonitoringEnabled()) { return powerEnabled(); }
        double posChange = motor.getCurrentPosition() - savePos;
        long timeChange = System.currentTimeMillis() - saveTime;
        savePos += posChange;
        saveTime += timeChange;
        double v_save = velocity;
        velocity = posChange / (double) timeChange * 1000;
        acceleration = (velocity - v_save) / (double) timeChange * 1000;
        for (LogTarget logTarget : loggingTargets) {
            if (logTarget == LogTarget.POWER) { continue; }
            String lt = logTarget.toString().toLowerCase();
            logData.put(lt, switch (logTarget) {
                case POSITION -> motor.getCurrentPosition();
                case TARGET_POSITION -> targetPosition;
                case POSITION_PID -> PIDController.get(id + "_T");
                case VELOCITY -> velocity;
                case TARGET_VELOCITY -> targetVelocity;
                case VELOCITY_PID -> PIDController.get(id + "_V");
                case ACCELERATION -> acceleration;
                default -> 0.0;
            });
        }
        return true;
    }
    
    /// {@inheritDoc}
    /// @return The unix time in milliseconds when the ping was received
    @Override
    public long ping() { return System.currentTimeMillis(); }

    /// {@inheritDoc}
    /// @return The wrapped motor
    @Override
    public Object getSelf() {
        return motor;
    }

    /// Returns the id of this motor controller
    /// @return The id of this motor controller
    @NonNull
    @Override
    public String toString() { return id; }
}
