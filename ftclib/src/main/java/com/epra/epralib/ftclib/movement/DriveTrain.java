package com.epra.epralib.ftclib.movement;

import androidx.annotation.NonNull;

import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Point;
import com.epra.epralib.ftclib.math.geometry.Vector;

import java.util.Set;

import java.util.HashMap;
import java.util.Map;

import com.epra.epralib.ftclib.math.geometry.Angle;

/**Coordinates motors in order to create cohesive robot motion.
 * This class can be used for a variable number of motors for several drive types.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class DriveTrain {
    /**All the orientations a motor can be in relative to the DriveTrain.*/
    public static enum Orientation {
        RIGHT (new Angle(0.0)),
        LEFT (new Angle(0.0)),
        FRONT (new Angle(90.0)),
        BACK (new Angle(90.0)),
        RIGHT_FRONT (new Angle(315.0)),
        RIGHT_BACK (new Angle(45.0)),
        LEFT_FRONT (new Angle(45.0)),
        LEFT_BACK (new Angle(315.0));

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

    private Angle target = new Angle(0);
    private Pose targetPose = new Pose(new Point(0,0), new Angle(0.0));
    private Pose lastTargetPose = new Pose(new Point(0,0), new Angle(0.0));
    private double toleranceMultiplier = 1.0;
    private PIDController anglePID = new PIDController(1, 0, 0);
    private PIDController pointPID = new PIDController(1, 0, 0);

    /**Coordinates motors in order to create cohesive robot motion.
     * This class can be used for a variable number of motors for several drive types.
     *
     * @param motorNames   Names that will be associated with each motor. Defaults to tank drive.
     * @param motors       DcMotorExs that will be used by the DriveTrain.
     * @param orientations The orientation of each motor.
     */
    public DriveTrain(String[] motorNames, MotorController[] motors, Orientation[] orientations) {
        for (int i = 0; i < Math.min(motorNames.length, motors.length); i++) {
            motor.put(motorNames[i], motors[i]);
            power.put(motorNames[i], 0.0);
            pos.put(motorNames[i], 0);
            orientation.put(motorNames[i], orientations[i]);
        }
        driveType = DriveType.TANK;
    }

    /**Coordinates motors in order to create cohesive robot motion.
     * This class can be used for a variable number of motors for several drive types.
     * @param motorNames   Names that will be associated with each motor.
     * @param motors       DcMotorExs that will be used by the DriveTrain.
     * @param orientations The orientation of each motor.
     * @param driveTypeIn  The drive type to be used.
     */
    public DriveTrain(String[] motorNames, MotorController[] motors, Orientation[] orientations, DriveType driveTypeIn) {
        for (int i = 0; i < Math.min(motorNames.length, motors.length); i++) {
            motor.put(motorNames[i], motors[i]);
            power.put(motorNames[i], 0.0);
            pos.put(motorNames[i], 0);
            orientation.put(motorNames[i], orientations[i]);
        }
        driveType = driveTypeIn;
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
        mecanumDrive(powerRightX, (float) vectorLeft.toPoint().y, (float) vectorLeft.toPoint().x);
    }
    /**
     * Field Oriented holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 10/31/2024.
     * @param powerRightX X position of the right joystick.
     * @param vectorLeft A vector representing the left joystick.
     * @param heading The angle of the robot relative to the field.
     *  */
    public void fieldOrientedMecanumDrive(float powerRightX, Vector vectorLeft, Angle heading) {
        mecanumDrive(powerRightX, new Vector(vectorLeft.getLength(), Geometry.subtract(vectorLeft, heading)));
    }

    /**
     * Field Oriented holonomic drive with mecanum wheels. Left stick moves the robot, right stick X rotates the robot. Created 11/26/2024.
     * @param vectorRight A vector representing the right joystick.
     * @param vectorLeft A vector representing the left joystick.
     * @param heading The angle of the robot relative to the field.
     * @param angleTolerance The tolerance for reaching the target angle as a positive double. If this is set to 0.0 the pid will run indefinitely.
     * @param haltAtTarget If true the motors will halt once the target is reached within the set tolerance.
     * @return True if the robot has reached the target angle, false if not.
     *  */
    public boolean fieldOrientedMecanumDrive(Vector vectorRight, Vector vectorLeft, Angle heading, double angleTolerance, boolean haltAtTarget) {
        if (vectorRight.getLength() > 0.25) { target.setRadian(vectorRight.getRadian()); }
        float rightPower = (float) anglePID.runPIDAngle(heading, target);
        if (Math.abs(rightPower) > angleTolerance) {
            fieldOrientedMecanumDrive(rightPower, vectorLeft, heading);
        } else {
            if (haltAtTarget) { fieldOrientedMecanumDrive(0, vectorLeft, heading); }
            anglePID.reset();
            return true;
        }
        return false;
    }

    /**Sets the target Pose of the PID. When running posPIDMecanumDrive the DriveTrain will attempt to move the robot to this position.
     * @param target Pose to test the target to.*/
    public void setTargetPose(Pose target) {
        if (target.point.x != targetPose.point.x || target.point.y != targetPose.point.y || target.angle.getDegree() != targetPose.angle.getDegree()) {
            lastTargetPose = targetPose;
            targetPose = target;
        }
    }

    /**
     * Field Oriented holonomic drive with mecanum wheels. Uses PID loops to reach the target position set. Created 11/27/2024.
     * @param current The current position.
     * @param posTolerance The tolerance for reaching the target position as a double between 0.0 and 1.0. If this is set to 0.0 the pid will run indefinitely.
     * @param angleTolerance The tolerance for reaching the target angle as a positive double. If this is set to 0.0 the pid will run indefinitely.
     * @param maxPower The maximum power of the motors.
     * @param haltAtTarget If true the motors will halt once the target is reached within the set tolerance.
     * @return True if the robot has reached the target position, false if not.
     *  */
    public boolean posPIDMecanumDrive(@NonNull Pose current, double posTolerance, double angleTolerance, double maxPower, boolean haltAtTarget) {
        Vector vectorLeft = pointPID.runPIDPoint(current.point, targetPose.point);
        Vector vectorRight = new Vector(1.0, targetPose.angle);
        boolean b = fieldOrientedMecanumDrive(vectorRight, new Vector(Math.min(maxPower, vectorLeft.getLength()),vectorLeft), current.angle, angleTolerance, haltAtTarget);
        if (Geometry.pythagorean(current.point, targetPose.point) <= getAbsolutePosTolerance(posTolerance) && b) {
            if (haltAtTarget) { mecanumDrive(0, 0, 0); }
            pointPID.reset();
            return true;
        }
        return false;
    }

    /**Finds the true tolerance to compare to the pos PID loop.
     * @param posTolerance A tolerance value between 0.0 and 1.0.
     * @return The true pos tolerance value.*/
    public double getAbsolutePosTolerance(double posTolerance) {
        double dst = Geometry.pythagorean(targetPose.point, lastTargetPose.point);
        return Math.abs(posTolerance * dst);
    }

    /**Tunes the PID loop used to reach the target angle.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneAnglePID(double k_p, double k_i, double k_d) {
        anglePID.tuneP(k_p);
        anglePID.tuneI(k_i);
        anglePID.tuneD(k_d);
    }

    /**Tunes the PID loop used to reach the target position.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tunePointPID(double k_p, double k_i, double k_d) {
        pointPID.tuneP(k_p);
        pointPID.tuneI(k_i);
        pointPID.tuneD(k_d);
        toleranceMultiplier = 1.0 / k_p;
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