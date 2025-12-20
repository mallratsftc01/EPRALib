package com.epra.epralib.ftclib.movement;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;

import java.util.ArrayList;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.storage.autonomous.DriveTrainAutoModule;

/// Coordinates multiple drive [Motor]s for cohesive motion.
///
/// Has several [DriveType]s and can control any number of [MotorController]s.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class DriveTrain {
    /// Signifies the location of a drive [Motor] on the robot.
    ///
    /// These positions are useful when coordinating different [DriveType]s.
    public enum Orientation {
        /// Signifies that a [Motor] is on the right side of the robot.
        RIGHT (Angle.degree(0.0)),
        /// Signifies that a [Motor] is on the left side of the robot.
        LEFT (Angle.degree(0.0)),
        /// Signifies that a [Motor] is on the front side of the robot.
        FRONT (Angle.degree(90.0)),
        /// Signifies that a [Motor] is on the back side of the robot.
        BACK (Angle.degree(90.0)),
        /// Signifies that a [Motor] is on the front right corner of the robot.
        RIGHT_FRONT (Angle.degree(315.0)),
        /// Signifies that a [Motor] is on the back right corner of the robot.
        RIGHT_BACK (Angle.degree(45.0)),
        /// Signifies that a [Motor] is on the front left corner of the robot.
        LEFT_FRONT (Angle.degree(45.0)),
        /// Signifies that a [Motor] is on the back left corner of the robot.
        LEFT_BACK (Angle.degree(315.0));

        final Angle angle;

        /// Signifies the location of a drive [Motor] on the robot.
        ///
        /// These positions are useful when coordinating different [DriveType]s.
        /// @param a The angle of the motor relative to the robot.
        Orientation(Angle a) { angle = a; }

        /// Returns if the orientation is on the right side of the robot.
        ///
        /// [#RIGHT], [#RIGHT_FRONT], and [#RIGHT_BACK] are considered right.
        /// @param o An orientation
        /// @return If the orientation is considered right
        static boolean isRight(Orientation o) {
            return switch (o) {
                case RIGHT, RIGHT_BACK, RIGHT_FRONT -> true;
                default -> false;
            };
        }

        /// Returns if the orientation is on the left side of the robot.
        ///
        /// [#LEFT], [#LEFT_FRONT], and [#LEFT_BACK] are considered left.
        /// @param o An orientation
        /// @return If the orientation is considered left
        static boolean isLeft(Orientation o) {
            return switch (o) {
                case LEFT, LEFT_BACK, LEFT_FRONT -> true;
                default -> false;
            };
        }

        /// Returns if the orientation is on the front side of the robot.
        ///
        /// [#FRONT], [#RIGHT_FRONT], and [#LEFT_FRONT] are considered front.
        /// @param o An orientation
        /// @return If the orientation is considered front
        static boolean isFront(Orientation o) {
            return switch (o) {
                case FRONT, RIGHT_FRONT, LEFT_FRONT -> true;
                default -> false;
            };
        }

        /// Returns if the orientation is on the back side of the robot.
        ///
        /// [#BACK], [#RIGHT_BACK], and [#LEFT_BACK] are considered back.
        /// @param o An orientation
        /// @return If the orientation is considered back
        static boolean isBack(Orientation o) {
            return switch (o) {
                case BACK, LEFT_BACK, RIGHT_BACK -> true;
                default -> false;
            };
        }
    }

    /// Signifies the style of driving to be used by [#drive(double, double, double, double)].
    public enum DriveType {
        /// A style of driving where each joystick directly controls the [Motor]s on that side of the robot.
        TANK,
        /// A style of driving that only uses one joystick, where forwards and backwards move forwards and backwards
        /// and right and left rotate the robot.
        ARCADE,
        /// A joke driving style created by accident when attempting to create [#ARCADE]. **Non-functional**
        ZACHARIAN,
        /// A holonomic driving style that uses four omniwheels, on on each side of the robot.
        X,
        /// A holonomic driving style that uses four mecanum wheels.
        MECANUM,

    }

    private final Map<String, MotorController> motor = new HashMap<>();
    private final Map<String, Double> power = new HashMap<>();
    private final Map<String, Orientation> orientation = new HashMap<>();

    private final DriveType driveType;

    private final boolean positionControlEnabled;
    private Angle target = new Angle();
    private Pose targetPose = new Pose(new Vector(0,0), new Angle());
    private Pose lastTargetPose = new Pose(new Vector(0,0), new Angle());
    private Vector lastMotionVector = new Vector(0, 0);

    /// The function by which the [DriveTrain] will retrieve positional data.
    ///
    /// This information is only used by methods that attempt to drive to a specific location on the field,
    /// such as [#posPIDMecanumDrive(double, double, double)]. If you are not using those methods, you
    /// do not need to initialize a pose supplier.
    ///
    /// [Odometry#getPose()] is the most common pose supplier.
    private final Supplier<Pose> poseSupplier;
    /// The function by which the [DriveTrain] will retrieve change in position data.
    ///
    /// This information is only used by methods that attempt to drive to a specific location on the field,
    /// such as [#posPIDMecanumDrive(double, double, double)]. If you are not using those methods, you
    /// do not need to initialize a delta pose supplier.
    ///
    /// [Odometry#getDeltaPose()] is the most common delta pose supplier.
    private final Supplier<Vector> deltaPoseSupplier;

    /// Coordinates multiple drive [Motor]s for cohesive motion.
    ///
    /// Has several [DriveType]s and can control any number of [MotorController]s.
    /// Position control is enabled. PID loops are not activated.
    /// @param motors An array of drive motor controllers 
    /// @param orientations An array of orientations corresponding to the motor controllers
    /// @param poseSupplier A function for retrieving positional data
    /// @param deltaPoseSupplier A function for retrieving change in position data
    /// @param positionControl If position control is enabled
    /// @param driveType A drive type
    /// @see #poseSupplier
    /// @see #deltaPoseSupplier
    public DriveTrain(MotorController[] motors, Orientation[] orientations, Supplier<Pose> poseSupplier, Supplier<Vector> deltaPoseSupplier, boolean positionControl, DriveType driveType) {
        for (int i = 0; i < motors.length; i++) {
            motor.put(motors[i].toString(), motors[i]);
            power.put(motors[i].toString(), 0.0);
            orientation.put(motors[i].toString(), orientations[i]);
        }
        this.driveType = driveType;
        this.poseSupplier = poseSupplier;
        this.deltaPoseSupplier = deltaPoseSupplier;
        this.positionControlEnabled = positionControl;
    }

    /// Coordinates multiple drive [Motor]s for cohesive motion.
    ///
    /// Has several [DriveType]s and can control any number of [MotorController]s. Position control is disabled.
    /// @param motors An array of drive motor controllers
    /// @param orientations An array of orientations corresponding to the motor controllers
    /// @param driveType A drive type
    public DriveTrain(MotorController[] motors, Orientation[] orientations, DriveType driveType) {
        this(motors, orientations, () -> new Pose(new Vector(0,0), new Angle()), () -> new Vector(0,0), false, driveType);
    }

    /// A builder for a [DriveTrain].
    public static class Builder {
        private final ArrayList<MotorController> motors;
        private final ArrayList<Orientation> orientations;
        private DriveType driveType;
        private Supplier<Pose> poseSupplier;
        private Supplier<Vector> deltaPoseSupplier;
        private double kp_p, ki_p, kd_p;
        private double kp_a, ki_a, kd_a;
        private double kp_m, ki_m, kd_m;
        private boolean tuneP, tuneA, tuneM;

        /// A builder for a [DriveTrain].
        ///
        /// Starts with default parameters:
        /// - No [MotorController]s or [Orientation]s.
        /// - [DriveType#TANK].
        /// - Both poseSupplier and deltaPoseSupplier are constant.
        /// - PID loops are not initialized.
        ///
        /// @see #motor(MotorController, Orientation)
        /// @see #driveType(DriveType)
        /// @see #poseSupplier(Supplier)
        /// @see #deltaPoseSupplier(Supplier)
        /// @see #anglePIDConstants(double, double, double)
        /// @see #pointPIDConstants(double, double, double)
        /// @see #motionPIDConstants(double, double, double)
        /// @see #build()
        public Builder() {
            this.motors = new ArrayList<>();
            this.orientations = new ArrayList<>();
            this.driveType = DriveType.TANK;
            this.poseSupplier = () -> null;
            this.deltaPoseSupplier = () -> null;
            tuneP = tuneA = tuneM = false;
        }
        /// Adds a [MotorController] and corresponding [Orientation] to the [DriveTrain].
        ///
        /// Drive motors should be used for the drive train, not servos.
        /// @param motor A motor controller
        /// @param orientation The orientation of the motor on the robot
        /// @return This builder
        public Builder motor(MotorController motor, Orientation orientation) {
            this.motors.add(motor);
            this.orientations.add(orientation);
            return this;
        }
        /// Adds a [MotorController] to the [DriveTrain].
        ///
        /// Only works for motor controllers that have an [Orientation] initialized, will have no effect otherwise.
        /// Drive motors should be used for the drive train, not servos.
        /// @param motor A motor controller
        /// @return This builder
        public Builder motor(MotorController motor) {
            if (motor.getDriveOrientation() == null) { return this; }
            this.motors.add(motor);
            this.orientations.add(motor.getDriveOrientation());
            return this;
        }
        /// Sets the [DriveType] for the [DriveTrain].
        /// @param driveType A drive type
        /// @return This builder
        public Builder driveType(DriveType driveType) {
            this.driveType = driveType;
            return this;
        }
        /// Sets the function by which the [DriveTrain] will retrieve positional data.
        /// 
        /// This information is only used by methods that attempt to drive to a specific location on the field,
        /// such as [#posPIDMecanumDrive(double, double, double)]. If you are not using those methods, you
        /// do not need to initialize a pose supplier. If this or [#deltaPoseSupplier(Supplier)] are not initialized,
        /// position control will be disabled.
        /// 
        /// [Odometry#getPose()] is the most common pose supplier.
        /// @param poseSupplier A pose supplier
        /// @return This builder
        public Builder poseSupplier(Supplier<Pose> poseSupplier) {
            this.poseSupplier = poseSupplier;
            return this;
        }
        /// Sets the function by which the [DriveTrain] will retrieve change in position data.
        ///
        /// This information is only used by methods that attempt to drive to a specific location on the field,
        /// such as [#posPIDMecanumDrive(double, double, double)]. If you are not using those methods, you
        /// do not need to initialize a delta pose supplier. If this or [#poseSupplier(Supplier)] are not initialized,
        /// position control will be disabled.
        ///
        /// [Odometry#getDeltaPose()] is the most common delta pose supplier.
        /// @param deltaPoseSupplier A delta pose supplier
        /// @return This builder
        public Builder deltaPoseSupplier(Supplier<Vector> deltaPoseSupplier) {
            this.deltaPoseSupplier = deltaPoseSupplier;
            return this;
        }

        /// Tunes and initializes the PID loop used to rotate to a target angle in
        /// [#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)] and [#posPIDMecanumDrive(double, double, double, boolean)] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param kp The `p` constant
        /// @param ki The `i` constant
        /// @param kd The `d` constant
        /// @return This builder
        ///
        /// @see PIDController
        public Builder anglePIDConstants(double kp, double ki, double kd) {
            kp_a = kp; ki_a = ki; kd_a = kd;
            tuneA = true;
            return this;
        }
        /// Tunes and initializes the PID loop used to rotate to a target angle in
        /// [#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)] and [#posPIDMecanumDrive(double, double, double, boolean)] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param pidGains A record with instructions to tune the PID loop
        /// @return This builder
        ///
        /// @see PIDController
        public Builder anglePIDConstants(PIDGains pidGains) {
            kp_a = pidGains.kp(); ki_a = pidGains.ki(); kd_a = pidGains.kd();
            tuneA = true;
            return this;
        }
        /// Tunes and initializes the PID loop used to move to a target point in [#posPIDMecanumDrive(double, double, double, boolean)] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param kp The `p` constant
        /// @param ki The `i` constant
        /// @param kd The `d` constant
        /// @return This builder
        ///
        /// @see PIDController
        public Builder pointPIDConstants(double kp, double ki, double kd) {
            kp_p = kp; ki_p = ki; kd_p = kd;
            tuneP = true;
            return this;
        }
        /// Tunes and initializes the PID loop used to move to a target point in [#posPIDMecanumDrive(double, double, double, boolean)] using
        /// [PIDController#tune(String, PIDGains)].
        /// @param pidGains A record with instructions to tune the PID loop
        /// @return This builder
        ///
        /// @see PIDController
        public Builder pointPIDConstants(PIDGains pidGains) {
            kp_p = pidGains.kp(); ki_p = pidGains.ki(); kd_p = pidGains.kd();
            tuneP = true;
            return this;
        }
        /// Tunes and initializes the PID loop used to reach the target motion vector in [#posPIDMecanumDrive(double, double, double)] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param kp The `p` constant
        /// @param ki The `i` constant
        /// @param kd The `d` constant
        /// @return This builder
        ///
        /// @see PIDController
        public Builder motionPIDConstants(double kp, double ki, double kd) {
            kp_m = kp; ki_m = ki; kd_m = kd;
            tuneM = true;
            return this;
        }
        /// Tunes and initializes the PID loop used to reach the target motion vector in [#posPIDMecanumDrive(double, double, double)] using
        /// [PIDController#tune(String, double, double, double)].
        /// @param pidGains A record with instructions to tune the PID loop
        /// @return This builder
        ///
        /// @see PIDController
        public Builder motionPIDConstants(PIDGains pidGains) {
            kp_m = pidGains.kp(); ki_m = pidGains.ki(); kd_m = pidGains.kd();
            tuneM = true;
            return this;
        }
        /// Builds a [DriveTrain] from this builder.
        /// @return The drive train built by this builder
        public DriveTrain build() {
            DriveTrain dt = new DriveTrain(motors.toArray(new MotorController[0]), orientations.toArray(new Orientation[0]),
                    (poseSupplier == null) ? poseSupplier : () -> new Pose(new Vector(0, 0), new Angle()),
                    (deltaPoseSupplier == null) ? deltaPoseSupplier : () -> new Vector(0, 0),
                    (deltaPoseSupplier != null && poseSupplier != null),
                    driveType);
            if (tuneP) dt.tunePointPID(kp_p, ki_p, kd_p);
            if (tuneA) dt.tuneAnglePID(kp_a, ki_a, kd_a);
            if (tuneM) dt.tuneMotionPID(kp_m, ki_m, kd_m);
            return dt;
        }
    }

    /// A style of driving where each power input directly powers all motors on that side of the robot.
    /// @param powerRight The power for the right side
    /// @param powerLeft The power for the left side
    ///
    /// @see DriveType#TANK
    public void tankDrive(double powerRight, double powerLeft) {
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            if (Orientation.isRight(entry.getValue())) {
                power.replace(entry.getKey(), powerRight);
            } else if (Orientation.isLeft(entry.getValue())) {
                power.replace(entry.getKey(), powerLeft);
            }
        }
        setMotorPowers();
    }

    /// A style of driving where one power input controls forward and backwards motion while the other controls turning.
    /// @param powerX The power input that controls turning
    /// @param powerY The power input that controls forwards and backwards motion
    ///
    /// @see DriveType#ARCADE
    public void arcadeDrive(double powerX, double powerY) {
        double powerRight = powerY + powerX;
        double powerLeft = powerX - powerY;
        powerLeft = (powerY > -0.5 && powerX < 0.5) ? 0 : powerLeft;
        powerRight = (powerX > -0.1 && powerY < 0.1) ? powerLeft : powerRight;
        tankDrive(powerRight, powerLeft);
    }

     /// A joke drive created by accident while trying to create the arcade drive.
     ///
     /// Created 2/12/2022.
     /// @param powerRightX X position of the right joystick.
     /// @param powerLeftX  X position of the left joystick.
     /// @param powerRightY Y position of the right joystick.
     /// @param powerLeftY  Y position of the left joystick.
     ///
     /// @see DriveType#ZACHARIAN
    public void zacharianDrive(double powerRightX, double powerLeftX, double powerRightY, double powerLeftY) {
        double powerRight = powerRightY + powerRightX;
        double powerLeft = powerLeftY - powerLeftX;
        tankDrive(powerRight, powerLeft);
    }

    /// A holonomic driving style that uses four omniwheels, one on each side of the robot.
    ///
    /// Created 9/17/2022.
    /// @param powerRightX The power input that controls turning
    /// @param powerLeftY The power input that controls forward-backwards motion
    /// @param powerLeftX The power input that controls side-to-side motion
    ///
    /// @see DriveType#X
    public void xDrive(double powerRightX, double powerLeftY, double powerLeftX) {
        double powerVar = powerRightX * 0.25;
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            if (Orientation.isRight(entry.getValue())) {
                power.replace(entry.getKey(), powerLeftY + powerVar);
            } else if (Orientation.isLeft(entry.getValue())) {
                power.replace(entry.getKey(), powerLeftX - powerVar);
            } else if (Orientation.isFront(entry.getValue())) {
                power.replace(entry.getKey(), powerLeftX + powerVar);
            } else if (Orientation.isBack(entry.getValue())) {
                power.replace(entry.getKey(), powerLeftY - powerVar);
            }
        }
        setMotorPowers();
    }

    /// A holonomic driving style that uses four mecanum wheels.
    ///
    /// Created 9/24/2022.
    /// @param powerRightX The power input that controls turning
    /// @param powerLeftY The power input that controls forward-backwards motion
    /// @param powerLeftX The power input that controls side-to-side motion
    ///
    /// @see DriveType#MECANUM
    /// @see #fieldOrientedMecanumDrive(double, Vector)
    /// @see #fieldOrientedMecanumDrive(double, Vector, Angle)
    /// @see #fieldOrientedMecanumDrive(Vector, Vector, double, boolean)
    public void mecanumDrive(double powerRightX, double powerLeftY, double powerLeftX) {
        double denominator = Math.max(Math.abs(powerLeftY) + Math.abs(powerLeftX) + Math.abs(powerRightX), 1);
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            switch (entry.getValue()) {
                case RIGHT_FRONT ->
                        power.replace(entry.getKey(), (-1 * powerLeftY - powerLeftX + powerRightX) / denominator);
                case LEFT_FRONT ->
                        power.replace(entry.getKey(), (-1 * powerLeftY + powerLeftX - powerRightX) / denominator);
                case RIGHT_BACK ->
                        power.replace(entry.getKey(), (-1 * powerLeftY + powerLeftX + powerRightX) / denominator);
                case LEFT_BACK ->
                        power.replace(entry.getKey(), (-1 * powerLeftY - powerLeftX - powerRightX) / denominator);
            }
        }
        setMotorPowers();
    }
    /// A holonomic driving style that uses four mecanum wheels.
    ///
    /// Created 9/24/2022.
    /// @param powerRightX The power input that controls turning
    /// @param vectorLeft A vector that controls linear motion
    ///
    /// @see DriveType#MECANUM
    /// @see #fieldOrientedMecanumDrive(double, Vector)
    /// @see #fieldOrientedMecanumDrive(double, Vector, Angle)
    /// @see #fieldOrientedMecanumDrive(Vector, Vector, double, boolean)
    public void mecanumDrive(double powerRightX, Vector vectorLeft) {
        mecanumDrive(powerRightX, vectorLeft.y(), vectorLeft.x());
    }

    /// A field-oriented holonomic driving style that uses four mecanum wheels.
    ///
    /// Created 10/31/2024.
    /// @param powerRightX The power input that controls turning
    /// @param vectorLeft A vector that controls linear motion
    /// @param heading The heading angle of the robot, used to correctly orient the drive
    ///
    /// @see DriveType#MECANUM
    /// @see #mecanumDrive(double, Vector)
    /// @see #mecanumDrive(double, double, double)
    public void fieldOrientedMecanumDrive(double powerRightX, Vector vectorLeft, Angle heading) {
        mecanumDrive(powerRightX, new Vector(vectorLeft.length(), Geometry.subtract(vectorLeft.theta(), heading)));
    }

    /// A field-oriented holonomic driving style that uses four mecanum wheels.
    ///
    /// Uses the [#poseSupplier] to find the heading angle to correctly orient the drive.
    ///
    /// Created 10/31/2024.
    /// @param powerRightX The power input that controls turning
    /// @param vectorLeft A vector that controls linear motion
    ///
    /// @see DriveType#MECANUM
    /// @see #mecanumDrive(double, Vector)
    /// @see #mecanumDrive(double, double, double)
    public void fieldOrientedMecanumDrive(double powerRightX, Vector vectorLeft) {
        fieldOrientedMecanumDrive(powerRightX, vectorLeft, poseSupplier.get().angle);
    }

    /// A field-oriented holonomic driving style that uses four mecanum wheels.
    ///
    /// Uses the [#poseSupplier] to find the heading angle to correctly orient the drive.
    ///
    /// Uses a PID loop to move to the target angle. Will have no effect and return `False` if the
    /// angle PID is not initialized. (Use [#tuneAnglePID(double, double, double)] to initialize.)
    /// PID loops must be updated frequently with [PIDController#update()] or this function will be ineffective.
    ///
    /// The tolerance controls how precise the motor must be in reaching the target.
    /// A tolerance of 0 or less will result in the motor never reaching the target.
    /// A tolerance of 1 or more will result in the motor never moving.
    ///
    /// Created 11/26/2024.
    /// @param vectorRight A vector whose angle will be used to orient the robot in that direction, relative to the field
    /// @param vectorLeft A vector that controls linear motion
    /// @param angleTolerance The tolerance for rotation as a number between 0 and 1
    /// @param haltAtTarget If the PID loop should be idled once the target angle is reached
    /// @return If the target angle has been reached
    ///
    /// @see DriveType#MECANUM
    /// @see #mecanumDrive(double, Vector)
    /// @see #mecanumDrive(double, double, double)
    /// @see PIDController
    public boolean fieldOrientedMecanumDrive(Vector vectorRight, Vector vectorLeft, double angleTolerance, boolean haltAtTarget) {
        if (!PIDController.hasPID("DriveTrain_A")) { return false; }
        PIDController.activate("DriveTrain_A");
        Angle heading = poseSupplier.get().angle;
        if (vectorRight.length() > 0.25) { target = vectorRight.theta(); }
        float rightPower = (float) PIDController.get("DriveTrain_A");
        if (Math.abs(rightPower) > angleTolerance) {
            fieldOrientedMecanumDrive(rightPower, vectorLeft, heading);
        } else {
            if (haltAtTarget) { fieldOrientedMecanumDrive(0, vectorLeft, heading); }
            PIDController.idle("DriveTrain_A");
            return true;
        }
        return false;
    }

    /// Sets a target [Pose] for the robot to drive towards using [#posPIDMecanumDrive(double, double, double)].
    public void setTargetPose(Pose target) {
        if (target.pos.x() != targetPose.pos.x() || target.pos.y()!= targetPose.pos.y() || target.angle.degree() != targetPose.angle.degree()) {
            lastTargetPose = targetPose;
            targetPose = target;
            PIDController.activate("DriveTrain_P");
            PIDController.activate("DriveTrain_M");
        }
    }

    /// An autonomous holonomic driving system that will use PID loops and a mecanum drive base to move the robot to the target position
    /// set in [#setTargetPose(Pose)].
    ///
    /// Uses the [#poseSupplier] to find the heading angle to correctly orient the drive and the current position of the
    /// robot on the field. Will return `False` if position control is not enabled.
    ///
    /// Uses a PID loop to move to the target position. Will have no effect and return `False` if the
    /// angle PID or point PID is not initialized. (Use [#tuneAnglePID(double, double, double)] and [#tunePointPID(double, double, double)] to initialize.)
    /// PID loops must be updated frequently with [PIDController#update()] or this function will be ineffective.
    ///
    /// The tolerance controls how precise the motor must be in reaching the target.
    /// A tolerance of 0 or less will result in the motor never reaching the target.
    /// A tolerance of 1 or more will result in the motor never moving.
    ///
    /// Created 11/27/2024.
    /// @param posTolerance The tolerance for moving to the target position as a number between 0 and 1
    /// @param angleTolerance The tolerance for rotation as a number between 0 and 1
    /// @param maxPower The absolute maximum power the drive train can use for each motor to reach
    /// the target position as a number between 0 and 1
    /// @param haltAtTarget If the PID loop should be idled once the target angle is reached
    /// @return If the target position has been reached
    ///
    /// @see DriveType#MECANUM
    /// @see PIDController
    /// @see #posPIDMecanumDrive(double, double, double)
    /// @see #posPIDMecanumDrive(DriveTrainAutoModule)
    public boolean posPIDMecanumDrive(double posTolerance, double angleTolerance, double maxPower, boolean haltAtTarget) {
        if (!positionControlEnabled || !PIDController.hasPID("DriveTrain_P") || !PIDController.hasPID("DriveTrain_A")) { return false; }
        PIDController.idle("DriveTrain_M");
        Pose current = poseSupplier.get();
        Vector vectorLeft = new Vector(PIDController.get("DriveTrain_P"), new Vector(-1 * (targetPose.pos.y() - current.pos.y()), (targetPose.pos.x() - current.pos.x())).theta());
        Vector vectorRight = new Vector(1.0, targetPose.angle);
        boolean b = fieldOrientedMecanumDrive(vectorRight, new Vector(Math.min(maxPower, vectorLeft.length()), vectorLeft.theta()), angleTolerance, haltAtTarget);
        if (Geometry.subtract(current.pos, targetPose.pos).length() <= getAbsolutePosTolerance(posTolerance) && b) {
            if (haltAtTarget) { mecanumDrive(0, 0, 0); }
            PIDController.idle("DriveTrain_P");
            return true;
        }
        return false;
    }

    /// Calculates the non-normalized position tolerance for [#posPIDMecanumDrive(double, double, double, boolean)]
    /// from the normalized tolerance.
    /// @param posTolerance A position tolerance as a number between 0 and 1
    /// @return A non-normalized position tolerance
    private double getAbsolutePosTolerance(double posTolerance) {
        double dst = Geometry.subtract(targetPose.pos, lastTargetPose.pos).length();
        return Math.abs(posTolerance * dst);
    }

    /// An autonomous holonomic driving system that will use PID loops and a mecanum drive base to move the robot towards the target position
    /// set in [#setTargetPose(Pose)].
    ///
    /// **A less precise version of [#posPIDMecanumDrive(double, double, double, boolean)] that moves towards the
    /// target position but may not land on that position exactly. Meant for following a path rather than ending at a
    /// specific point.**
    ///
    /// Uses the [#poseSupplier] to find the heading angle to correctly orient the drive and the current position of the
    /// robot on the field. Will return `False` if position control is not enabled.
    ///
    /// Uses a PID loop to move towards the target position. Will have no effect and return `False` if the
    /// angle PID or motion PID is not initialized. (Use [#tuneAnglePID(double, double, double)] and [#tuneMotionPID(double, double, double)] to initialize.)
    /// PID loops must be updated frequently with [PIDController#update()] or this function will be ineffective.
    ///
    /// The tolerance controls how precise the motor must be in reaching the target.
    /// A tolerance of 0 or less will result in the motor never reaching the target.
    /// A tolerance of 1 or more will result in the motor never moving.
    ///
    /// Created 6/17/2025.
    /// @param posRange A range in **inches** around the target position for
    /// which the robot will be considered to have reached the target
    /// @param angleTolerance The tolerance for rotation as a number between 0 and 1
    /// @param maxPower The absolute maximum power the drive train can use for each motor to reach
    /// the target position as a number between 0 and 1
    /// @return If the target position has been reached
    ///
    /// @see DriveType#MECANUM
    /// @see PIDController
    /// @see #posPIDMecanumDrive(DriveTrainAutoModule)
    public boolean posPIDMecanumDrive(double posRange, double angleTolerance, double maxPower) {
        if (!positionControlEnabled || !PIDController.hasPID("DriveTrain_M") || !PIDController.hasPID("DriveTrain_A")) { return false; }
        PIDController.idle("DriveTrain_P");
        Pose current = poseSupplier.get();
        Vector deltaPos = deltaPoseSupplier.get();
        Vector targetVector = new Vector(current.pos, targetPose.pos);
        if (targetVector.length() < 1) { targetVector = new Vector(1, targetVector.theta()); }
        Vector delta = new Vector(lastMotionVector.length(), deltaPos.theta());
        lastMotionVector = Geometry.add(new Vector(PIDController.get("DriveTrain_M"), new Vector(-1 * (delta.y() - targetVector.y()), (delta.x() - targetVector.x())).theta()), lastMotionVector);
        if (lastMotionVector.length() > maxPower) { lastMotionVector = new Vector(maxPower, lastMotionVector.theta()); }
        Vector vectorRight = new Vector(1.0, targetPose.angle);
        fieldOrientedMecanumDrive(vectorRight, lastMotionVector, angleTolerance, false);
        boolean b = Math.abs(Geometry.subtract(targetPose.pos, current.pos).length()) <= posRange;
        if (b) { PIDController.idle("DriveTrain_M"); }
        return b;
    }

    /// An autonomous holonomic driving system that will use PID loops and a mecanum drive base to move the robot according
    /// to instructions in a [DriveTrainAutoModule].
    ///
    /// Uses either [#posPIDMecanumDrive(double, double, double, boolean)] or [#posPIDMecanumDrive(double, double, double)]
    /// according to the instructions.
    ///
    /// Uses the [#poseSupplier] to find the heading angle to correctly orient the drive and the current position of the
    /// robot on the field. Will return `False` if position control is not enabled.
    ///
    /// Uses a PID loop to move to the target position. Will have no effect and return `False` if the
    /// angle PID, point PID, or motion PID is not initialized.
    /// (Use [#tuneAnglePID(double, double, double)], [#tunePointPID(double, double, double)], and [#tuneMotionPID(double, double, double)] to initialize.)
    /// PID loops must be updated frequently with [PIDController#update()] or this function will be ineffective.
    ///
    /// @param driveTrainAutoModule An auto module with instructions for the drive train
    /// @return If the target position has been reached
    ///
    /// @see DriveType#MECANUM
    /// @see PIDController
    public boolean posPIDMecanumDrive(DriveTrainAutoModule driveTrainAutoModule) {
        setTargetPose(driveTrainAutoModule.targetPose());
        if (driveTrainAutoModule.usePrecision()) {
            return posPIDMecanumDrive(driveTrainAutoModule.posTolerance(), driveTrainAutoModule.angleTolerance(), driveTrainAutoModule.maxPower(), true);
        } else {
            return posPIDMecanumDrive(driveTrainAutoModule.posTolerance(), driveTrainAutoModule.angleTolerance(), driveTrainAutoModule.maxPower());
        }
    }

    /// Tunes and/or initializes the PID loop used to rotate to a target angle in
    /// [#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)] and [#posPIDMecanumDrive(double, double, double, boolean)] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    ///
    /// @see PIDController
    public void tuneAnglePID(double kp, double ki, double kd) { 
        if (PIDController.hasPID("DriveTrain_A")) {
            PIDController.tune("DriveTrain_A", kp, ki, kd);
        } else {
            PIDController.addPID("DriveTrain_A", kp, ki, kd, this::getAngleError, false);
        }
    }

    /// Tunes and/or initializes the PID loop used to rotate to a target angle in
    /// [#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)] and [#posPIDMecanumDrive(double, double, double, boolean)] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param pidGains A record with instructions to tune the PID loop
    ///
    /// @see PIDController
    public void tuneAnglePID(PIDGains pidGains) {
        if (PIDController.hasPID("DriveTrain_A")) {
            PIDController.tune("DriveTrain_A", pidGains);
        } else {
            PIDController.addPID("DriveTrain_A", pidGains, this::getAngleError, false);
        } 
    }

    /// Tunes and/or initializes the PID loop used to move to a target point in [#posPIDMecanumDrive(double, double, double, boolean)] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    ///
    /// @see PIDController
    public void tunePointPID(double kp, double ki, double kd) {
        if (PIDController.hasPID("DriveTrain_P")) {
            PIDController.tune("DriveTrain_P", kp, ki, kd);
        } else {
            PIDController.addPID("DriveTrain_P", kp, ki, kd, this::getPointError, false);
        }
    }
    /// Tunes and/or initializes the PID loop used to move to a target point in [#posPIDMecanumDrive(double, double, double, boolean)] using
    /// [PIDController#tune(String, PIDGains)].
    /// @param pidGains A record with instructions to tune the PID loop
    ///
    /// @see PIDController
    public void tunePointPID(PIDGains pidGains) {
        if (PIDController.hasPID("DriveTrain_P")) {
            PIDController.tune("DriveTrain_P", pidGains);
        } else {
            PIDController.addPID("DriveTrain_P", pidGains, this::getPointError, false);
        }
    }

    /// Tunes and/or initializes the PID loop used to reach the target motion vector in [#posPIDMecanumDrive(double, double, double)] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    ///
    /// @see PIDController
    public void tuneMotionPID(double kp, double ki, double kd) {
        if (PIDController.hasPID("DriveTrain_M")) {
            PIDController.tune("DriveTrain_M", kp, ki, kd);
        } else {
            PIDController.addPID("DriveTrain_M", kp, ki, kd, this::getMotionError, false);
        }
    }
    /// Tunes and/or initializes the PID loop used to reach the target motion vector in [#posPIDMecanumDrive(double, double, double)] using
    /// [PIDController#tune(String, double, double, double)].
    /// @param pidGains A record with instructions to tune the PID loop
    ///
    /// @see PIDController
    public void tuneMotionPID(PIDGains pidGains) {
        if (PIDController.hasPID("DriveTrain_M")) {
            PIDController.tune("DriveTrain_M", pidGains);
        } else {
            PIDController.addPID("DriveTrain_M", pidGains, this::getMotionError, false);
        }
    }

    /// Returns the error between the current [Angle] and the target [Angle].
    /// @return The angle error
    public double getAngleError() {
        return Geometry.subtract(target, poseSupplier.get().angle).radian();
    }

    /// Returns the error between the current position and the target position set in [#setTargetPose(Pose)].
    /// @return The position error
    public double getPointError() {
        return Geometry.subtract(targetPose.pos, poseSupplier.get().pos).length();
    }

    /// Returns the error between the current motion [Vector] and the target motion [Vector].
    /// @return The motion error
    public double getMotionError() {
        Pose current = poseSupplier.get();
        Vector deltaPos = deltaPoseSupplier.get();
        Vector targetVector = new Vector(current.pos, targetPose.pos);
        if (targetVector.length() < 1) { targetVector = new Vector(1, targetVector.theta()); }
        Vector delta = new Vector(lastMotionVector.length(), deltaPos.theta());
        return Geometry.subtract(delta, targetVector).length();
    }

    /// Using joystick inputs, drives with the [DriveType] of this drive train.
    /// @param powerRightX X position of the right joystick
    /// @param powerRightY Y position of the right joystick
    /// @param powerLeftX  X position of the left joystick
    /// @param powerLeftY  Y position of the left joystick
    /// @see Controller
    public void drive(double powerRightX, double powerRightY, double powerLeftX, double powerLeftY) {
        switch (driveType) {
            case TANK -> tankDrive(powerRightY, powerLeftY);
            case ARCADE -> arcadeDrive(powerLeftX, powerLeftY);
            case ZACHARIAN -> zacharianDrive(powerRightX, powerLeftX, powerRightY, powerLeftY);
            case X -> xDrive(powerRightX, powerLeftY, powerLeftX);
            case MECANUM -> mecanumDrive(powerRightX, powerLeftY, powerLeftX);
            default -> tankDrive(powerRightY, powerLeftY);
        }
        setMotorPowers();
    }
    /// Using joystick inputs, drives with the [DriveType] of this drive train.
    /// @param powerRight Vector position of the right joystick
    /// @param powerLeft  Vector position of the left joystick
    /// @see Controller
    public void drive(Vector powerRight, Vector powerLeft) {
        drive(powerRight.x(), powerRight.y(), powerLeft.x(), powerLeft.y());
    }

    /// Sets the power of each motor using the power values in [#power].
    private void setMotorPowers() {
        for (Map.Entry<String, Double> entry : power.entrySet()) {
            motor.get(entry.getKey()).setPower(entry.getValue());
        }
    }

    /// Calculates the [Vector] of motion for the specified [MotorController].
    ///
    /// A motion vector for a motor is a vector at the angle of the motor relative to the drive train and
    /// with a magnitude equal to the power that motor is currently running at.
    /// @param motorId The string id of a motor controller
    /// @return The motion vector of that motor
    /// @see #driveTrainVector()
    public Vector motorMotionVector(String motorId) { return new Vector(power.get(motorId), orientation.get(motorId).angle); }

    /// Calculates the [Vector] of motion for the entire drive train by
    /// combining the vectors of motion from each [MotorController] calculated using [#motorMotionVector(String)].
    /// @return The motion vector of the drive train
    public Vector driveTrainVector() {
        Vector v = new Vector(0, 0);
        for (Map.Entry<String, MotorController> entry : motor.entrySet()) {
            v = Geometry.add(v, motorMotionVector(entry.getKey()));
        }
        return v;
    }
}