package com.epra.epralib.ftclib.movement;

import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.math.geometry.Vector;

import java.util.Set;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.storage.DriveTrainAutoModule;
import com.epra.epralib.ftclib.storage.PIDGains;

/**Coordinates motors in order to create cohesive robot motion.
 * This class can be used for a variable number of motors for several drive types.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class DriveTrain {
    /**All the orientations a motor can be in relative to the DriveTrain.*/
    public static enum Orientation {
        RIGHT (Angle.degree(0.0)),
        LEFT (Angle.degree(0.0)),
        FRONT (Angle.degree(90.0)),
        BACK (Angle.degree(90.0)),
        RIGHT_FRONT (Angle.degree(315.0)),
        RIGHT_BACK (Angle.degree(45.0)),
        LEFT_FRONT (Angle.degree(45.0)),
        LEFT_BACK (Angle.degree(315.0));

        Angle angle;

        Orientation(Angle a) { angle = a;};

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is right, false if not.
         */
        static boolean ifRight(Orientation o) {
            return switch (o) {
                case RIGHT, RIGHT_BACK, RIGHT_FRONT -> true;
                default -> false;
            };
        }

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is left, false if not.
         */
        static boolean ifLeft(Orientation o) {
            return switch (o) {
                case LEFT, LEFT_BACK, LEFT_FRONT -> true;
                default -> false;
            };
        }

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is front, false if not.
         */
        static boolean ifFront(Orientation o) {
            return switch (o) {
                case FRONT, RIGHT_FRONT, LEFT_FRONT -> true;
                default -> false;
            };
        }

        /**
         * @param o Orientation to be tested.
         * @return True if the provided is back, false if not.
         */
        static boolean ifBack(Orientation o) {
            return switch (o) {
                case BACK, LEFT_BACK, RIGHT_BACK -> true;
                default -> false;
            };
        }
    }

    /**All of the drive types currently available.*/
    public static enum DriveType {
        TANK,
        ARCADE,
        ZACHARIAN,
        X,
        MECANUM,

    }

    /**
     * Map of all the motors.
     */
    private Map<String, MotorController> motor = new HashMap<>();
    /**
     * Map of all the motor powers.
     */
    private Map<String, Double> power = new HashMap<>();
    /**
     * Map of all motor positions.
     */
    private Map<String, Integer> pos = new HashMap<>();
    /**
     * Map of all orientations of motors.
     */
    private Map<String, Orientation> orientation = new HashMap<>();

    private DriveType driveType;
    private float wheelCircumference = 11.2192926146f; // 96mm diameter wheels circumference in inches
    private float gearRatio = 20;

    private Angle target = new Angle();
    private Pose targetPose = new Pose(new Vector(0,0), new Angle());
    private Pose lastTargetPose = new Pose(new Vector(0,0), new Angle());
    private Vector lastMotionVector = new Vector(0, 0);
    private double toleranceMultiplier = 1.0;

    private Supplier<Pose> poseSupplier;
    private Supplier<Vector> deltaPoseSupplier;

    /**Coordinates motors in order to create cohesive robot motion.
     * This class can be used for a variable number of motors for several drive types.
     *
     * @param motors       DcMotorExs that will be used by the DriveTrain.
     * @param orientations The orientation of each motor.
     */
    public DriveTrain(MotorController[] motors, Orientation[] orientations, Supplier<Pose> poseSupplier, Supplier<Vector> deltaPoseSupplier) {
        for (int i = 0; i < motors.length; i++) {
            motor.put(motors[i].toString(), motors[i]);
            power.put(motors[i].toString(), 0.0);
            pos.put(motors[i].toString(), 0);
            orientation.put(motors[i].toString(), orientations[i]);
        }
        driveType = DriveType.TANK;
        this.poseSupplier = poseSupplier;
        this.deltaPoseSupplier = deltaPoseSupplier;
        PIDController.addPID("DriveTrain_P", 1, 0, 0, this::getPointError, false);
        PIDController.addPID("DriveTrain_A", 1, 0, 0, this::getAngleError, false);
        PIDController.addPID("DriveTrain_V", 1, 0, 0, this::getVectorError, false);
    }

    /**Coordinates motors in order to create cohesive robot motion.
     * This class can be used for a variable number of motors for several drive types.
     * @param motors       DcMotorExs that will be used by the DriveTrain.
     * @param orientations The orientation of each motor.
     * @param driveTypeIn  The drive type to be used.
     */
    public DriveTrain( MotorController[] motors, Orientation[] orientations, Supplier<Pose> poseSupplier, Supplier<Vector> deltaPoseSupplier, DriveType driveTypeIn) {
        for (int i = 0; i < motors.length; i++) {
            motor.put(motors[i].toString(), motors[i]);
            power.put(motors[i].toString(), 0.0);
            pos.put(motors[i].toString(), 0);
            orientation.put(motors[i].toString(), orientations[i]);
        }
        driveType = driveTypeIn;
        this.poseSupplier = poseSupplier;
        this.deltaPoseSupplier = deltaPoseSupplier;
        PIDController.addPID("DriveTrain_P", 1, 0, 0, this::getPointError, false);
        PIDController.addPID("DriveTrain_A", 1, 0, 0, this::getAngleError, false);
        PIDController.addPID("DriveTrain_V", 1, 0, 0, this::getVectorError, false);
    }

    /**
     * Right power directly powers right motors, left power directly powers left motors.
     *
     * @param powerRight Power for right motors.
     * @param powerLeft  Power for left motors.
     */
    public void tankDrive(float powerRight, float powerLeft) {
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            if (Orientation.ifRight(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerRight);
            } else if (Orientation.ifLeft(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeft);
            }
        }
        setMotorPowers();
    }

    /**
     * Drive with only one joystick. Forward and backwards move forward and backwards, left and right turns.
     *
     * @param powerX The X position of the joystick.
     * @param powerY The Y position of the joystick.
     */
    public void arcadeDrive(float powerX, float powerY) {
        float powerRight = powerY + powerX;
        float powerLeft = powerY - powerX;
        powerLeft = (powerY > -0.5 && powerX < 0.5) ? 0 : powerLeft;
        powerRight = (powerX > -0.1 && powerY < 0.1) ? powerLeft : powerRight;
        tankDrive(powerRight, powerLeft);
    }

    /**
     * A joke drive created by accident while trying to create the arcade drive. Created 2/12/2022.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerRightY Y position of the right joystick.
     * @param powerLeftY  Y position of the left joystick.
     */
    public void zacharianDrive(float powerRightX, float powerLeftX, float powerRightY, float powerLeftY) {
        float powerRight = powerRightY + powerRightX;
        float powerLeft = powerLeftY - powerLeftX;
        tankDrive(powerRight, powerLeft);
    }

    /**
     * Holonomic drive with four omni wheels. Created 9/17/2022.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerLeftY  Y position of the left joystick.
     */
    public void xDrive(float powerRightX, float powerLeftX, float powerLeftY) {
        double powerVar = powerRightX * 0.25;
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            if (Orientation.ifRight(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftY + powerVar);
            } else if (Orientation.ifLeft(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftX - powerVar);
            } else if (Orientation.ifFront(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftX + powerVar);
            } else if (Orientation.ifBack(entry.getValue())) {
                power.replace(entry.getKey(), (double) powerLeftY - powerVar);
            }
        }
        setMotorPowers();
    }

    /**
     * Holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 9/24/2022.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerLeftY  Y position of the left joystick.
     */
    public void mecanumDrive(float powerRightX, float powerLeftX, float powerLeftY) {
        double denominator = Math.max(Math.abs(powerLeftY) + Math.abs(powerLeftX) + Math.abs(powerRightX), 1);
        for (Map.Entry<String, Orientation> entry : orientation.entrySet()) {
            switch (entry.getValue()) {
                case RIGHT_FRONT ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY + powerRightX + powerLeftX) / denominator);
                case LEFT_FRONT ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY + powerRightX - powerLeftX) / denominator);
                case RIGHT_BACK ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY - powerRightX + powerLeftX) / denominator);
                case LEFT_BACK ->
                        power.replace(entry.getKey(), (double) (-1 * powerLeftY - powerRightX - powerLeftX) / denominator);
            }
        }
        setMotorPowers();
    }
    /**
     * Holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 9/24/2022.
     * @param powerRightX X position of the right joystick.
     * @param vectorLeft A vector representing the left joystick.
     */
    public void mecanumDrive(float powerRightX, Vector vectorLeft) {
        mecanumDrive(powerRightX, (float) vectorLeft.y(), (float) vectorLeft.x());
    }
    /**
     * Field Oriented holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 10/31/2024.
     * @param powerRightX X position of the right joystick.
     * @param vectorLeft A vector representing the left joystick.
     * @param heading The angle of the robot relative to the field.
     *  */
    public void fieldOrientedMecanumDrive(float powerRightX, Vector vectorLeft, Angle heading) {
        mecanumDrive(powerRightX, new Vector(vectorLeft.length(), Geometry.subtract(vectorLeft.theta(), heading)));
    }

    /**
     * Field Oriented holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 11/26/2024.
     * @param vectorRight A vector representing the right joystick.
     * @param vectorLeft A vector representing the left joystick.
     * @param angleTolerance The tolerance for reaching the target angle as a positive double. If this is set to 0.0 the pid will run indefinitely.
     * @param haltAtTarget If true the motors will halt once the target is reached within the set tolerance.
     * @return True if the robot has reached the target angle, false if not.
     *  */
    public boolean fieldOrientedMecanumDrive(Vector vectorRight, Vector vectorLeft, double angleTolerance, boolean haltAtTarget) {
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

    /**Sets the target Pose of the PID. When running posPIDMecanumDrive the DriveTrain will attempt to move the robot to this position.
     * @param target Pose to test the target to.*/
    public void setTargetPose(Pose target) {
        if (target.pos.x()!= targetPose.pos.x()|| target.pos.y()!= targetPose.pos.y()|| target.angle.degree() != targetPose.angle.degree()) {
            lastTargetPose = targetPose;
            targetPose = target;
            PIDController.activate("DriveTrain_P");
        }
    }

    /**
     * Field Oriented holonomic drive with mecanum wheels. Uses PID loops to reach the target position set. Created 11/27/2024.
     * @param posTolerance The tolerance for reaching the target position as a double between 0.0 and 1.0. If this is set to 0.0 the pid will run indefinitely.
     * @param angleTolerance The tolerance for reaching the target angle as a positive double. If this is set to 0.0 the pid will run indefinitely.
     * @param maxPower The maximum power of the motors.
     * @param haltAtTarget If true the motors will halt once the target is reached within the set tolerance.
     * @return True if the robot has reached the target position, false if not.
     *  */
    public boolean posPIDMecanumDrive(double posTolerance, double angleTolerance, double maxPower, boolean haltAtTarget) {
        Pose current = poseSupplier.get();
        Vector vectorLeft = new Vector(PIDController.get("DriveTrain_P"), new Vector(-1 * (targetPose.pos.y()- current.pos.y()), (targetPose.pos.x()- current.pos.x())).theta());
        Vector vectorRight = new Vector(1.0, targetPose.angle);
        boolean b = fieldOrientedMecanumDrive(vectorRight, new Vector(Math.min(maxPower, vectorLeft.length()), vectorLeft.theta()), angleTolerance, haltAtTarget);
        if (Geometry.subtract(current.pos, targetPose.pos).length() <= getAbsolutePosTolerance(posTolerance) && b) {
            if (haltAtTarget) { mecanumDrive(0, 0, 0); }
            PIDController.idle("DriveTrain_P");
            return true;
        }
        return false;
    }

    /**Finds the true tolerance to compare to the pos PID loop.
     * @param posTolerance A tolerance value between 0.0 and 1.0.
     * @return The true pos tolerance value.*/
    public double getAbsolutePosTolerance(double posTolerance) {
        double dst = Geometry.subtract(targetPose.pos, lastTargetPose.pos).length();
        return Math.abs(posTolerance * dst);
    }

    /**Field Oriented holonomic drive with mecanum wheels. Uses PID loops to approach target position set. To be used for path following, not precise positioning. Created 6/17/2025.
     * @param posTolerance The range in inches around the target pose for which the method will return true as a positive double.
     * @param angleTolerance The tolerance for reaching the target angle as a positive double. If this is set to 0.0 the pid will run indefinitely.
     * @param maxPower The maximum power of the motors.
     * @return True if the robot has moved withing a radius of posTolerance of the target Vector, false if not.*/
    public boolean posPIDMecanumDrive(double posTolerance, double angleTolerance, double maxPower) {
        Pose current = poseSupplier.get();
        Vector deltaPos = deltaPoseSupplier.get();
        Vector targetVector = new Vector(current.pos, targetPose.pos);
        if (targetVector.length() < 1) { targetVector = new Vector(1, targetVector.theta()); }
        Vector delta = new Vector(lastMotionVector.length(), deltaPos.theta());
        lastMotionVector = Geometry.add(new Vector(PIDController.get("DriveTrain_P"), new Vector(-1 * (delta.y() - targetVector.y()), (delta.x() - targetVector.x())).theta()), lastMotionVector);
        if (lastMotionVector.length() > maxPower) { lastMotionVector = new Vector(maxPower, lastMotionVector.theta()); }
        Vector vectorRight = new Vector(1.0, targetPose.angle);
        fieldOrientedMecanumDrive(vectorRight, lastMotionVector, angleTolerance, false);
        boolean b = Math.abs(Geometry.subtract(targetPose.pos, current.pos).length()) <= posTolerance;
        if (b) { PIDController.idle("DriveTrain_V"); }
        return b;
    }

    /**
     * Field Oriented holonomic drive with mecanum wheels. Uses PID loops to reach the target position set. Created 11/27/2024.
     * @param driveTrainAutoModule A DriveTrainAutoModule that stores instructions for this DriveTrain.
     * @return True if the robot has reached the target position, false if not.
     *  */
    public boolean posPIDMecanumDrive(DriveTrainAutoModule driveTrainAutoModule) {
        setTargetPose(driveTrainAutoModule.targetPose());
        if (driveTrainAutoModule.usePrecision()) {
            return posPIDMecanumDrive(driveTrainAutoModule.posTolerance(), driveTrainAutoModule.angleTolerance(), driveTrainAutoModule.maxPower(), true);
        } else {
            return posPIDMecanumDrive(driveTrainAutoModule.posTolerance(), driveTrainAutoModule.angleTolerance(), driveTrainAutoModule.maxPower());
        }
    }

    /**Tunes the PID loop used to reach the target angle.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneAnglePID(double k_p, double k_i, double k_d) { PIDController.tune("DriveTrain_A", k_p, k_i, k_d); }

    /**Tunes the PID loop used to reach the target angle.
     * @param pidGains A PIDGains record containing the gains for a PIDController.
     */
    public void tuneAnglePID(PIDGains pidGains) { PIDController.tune("DriveTrain_A",pidGains); }

    /**Tunes the PID loop used to reach the target position.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tunePointPID(double k_p, double k_i, double k_d) {
        PIDController.tune("DriveTrain_P",k_p, k_i, k_d);
        toleranceMultiplier = 1.0 / k_p;
    }
    /**Tunes the PID loop used to reach the target position.
     * @param pidGains A PIDGains record containing the gains for a PIDController.
     */
    public void tunePointPID(PIDGains pidGains) { PIDController.tune("DriveTrain_P",pidGains); }

    /**Tunes the PID loop used to reach the target motion vector.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneVectorPID(double k_p, double k_i, double k_d) { PIDController.tune("DriveTrain_V",k_p, k_i, k_d); }
    /**Tunes the PID loop used to reach the target motion vector.
     * @param pidGains A PIDGains record containing the gains for a PIDController.
     */
    public void tuneVectorPID(PIDGains pidGains) { PIDController.tune("DriveTrain_V",pidGains); }

    /**Returns the error between the target angle and the current angle.
     * @return The error from the target angle.*/
    public double getAngleError() {
        return Geometry.subtract(target, poseSupplier.get().angle).radian();
    }

    /**Returns the error between the target angle and the current angle.
     * @return The error from the target angle.*/
    public double getPointError() {
        return Geometry.subtract(targetPose.pos, poseSupplier.get().pos).length();
    }

    public double getVectorError() {
        Pose current = poseSupplier.get();
        Vector deltaPos = deltaPoseSupplier.get();
        Vector targetVector = new Vector(current.pos, targetPose.pos);
        if (targetVector.length() < 1) { targetVector = new Vector(1, targetVector.theta()); }
        Vector delta = new Vector(lastMotionVector.length(), deltaPos.theta());
        return Geometry.subtract(delta, targetVector).length();
    }

    /**Uses a drive based on the DriveTrain's drive type.
     * @param powerRightX X position of the right joystick.
     * @param powerLeftX  X position of the left joystick.
     * @param powerRightY Y position of the right joystick.
     * @param powerLeftY  Y position of the left joystick.
     * */
    public void setDrivePower(float powerRightX, float powerLeftX, float powerRightY, float powerLeftY) {
        switch (driveType) {
            case TANK -> tankDrive(powerRightY, powerLeftY);
            case ARCADE -> arcadeDrive(powerLeftX, powerLeftY);
            case ZACHARIAN -> zacharianDrive(powerRightX, powerLeftX, powerRightY, powerLeftY);
            case X -> xDrive(powerRightX, powerLeftX, powerLeftY);
            case MECANUM -> mecanumDrive(powerRightX, powerLeftX, powerLeftY);
            default -> tankDrive(powerRightY, powerLeftY);
        }
        setMotorPowers();
    }

    /**Drives the robot using the dPad. Do not use if using any other drive method.
     * @param downPressed If down is pressed on the dPad.
     * @param upPressed If up is pressed on the dPad.
     * @param leftPressed If left is pressed on the dPad.
     * @param rightPressed If right is pressed on the dPad.*/
    public void dPadDrive(boolean upPressed, boolean downPressed, boolean leftPressed, boolean rightPressed) {
        if (downPressed) {
            setDrivePower(0, 0, -1, -1);
        } else if (upPressed) {
            setDrivePower(0, 0, 1, 1);
        } else if (leftPressed) {
            setDrivePower(-1, -1, 0, 0);
        } else if (rightPressed) {
            setDrivePower(1, 1, 0, 0);
        } else {
            setDrivePower(0, 0, 0, 0);
        }
    }

    /**Sets the power of each motor to the mapped motor power.*/
    private void setMotorPowers() {
        for (Map.Entry<String, Double> entry : power.entrySet()) {
            motor.get(entry.getKey()).setPower(entry.getValue());
        }
    }

    /**Updates all the pos values in the pos map.*/
    public void updatePos() {
        for (Map.Entry<String, MotorController> entry : motor.entrySet()) {
            pos.replace(entry.getKey(), entry.getValue().getCurrentPosition());
        }
    }
    /**@param motorName The name of the motor to search for.
     * @return The position of the specified motor.*/
    public int getPos(String motorName) {
        updatePos();
        return pos.get(motorName);
    }
    /**@return A set of all motor names and their positions.*/
    public Set<Map.Entry<String, Integer>> getPos() {
        updatePos();
        return pos.entrySet();
    }

    /**@param motorName The name of the motor.
     * @return The vector of motion of the motor.*/
    public Vector motorVector(String motorName) { return new Vector(power.get(motorName), orientation.get(motorName).angle); }

    /**@return The vector of motion of the DriveTrain as a combination of the vectors of motion of all the motors.*/
    public Vector driveTrainVector() {
        Vector v = new Vector(0, 0);
        for (Map.Entry<String, MotorController> entry : motor.entrySet()) {
            v = Geometry.add(v, motorVector(entry.getKey()));
        }
        return v;
    }
}