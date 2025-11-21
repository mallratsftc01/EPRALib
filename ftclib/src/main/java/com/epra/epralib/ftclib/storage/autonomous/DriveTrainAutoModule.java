package com.epra.epralib.ftclib.storage.autonomous;

import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;

public class DriveTrainAutoModule {

    private final double targetX;
    private final double targetY;
    private final double targetAngle;
    private final double posTolerance;
    private final double angleTolerance;
    private final double maxPower;
    private final boolean usePrecision;
    private final double weight;

    /**
     * A record that stores instructions for the DriveTrain during auto.
     * <p>
     * JSON Format:
     * <pre><code>
     *     {
     *         "targetX": float,
     *         "targetY": float,
     *         "targetAngle": float,
     *         "posTolerance": float,
     *         "angleTolerance": float,
     *         "maxPower": float,
     *         "usePrecision": boolean,
     *         "weight": float
     *     } </code></pre>
     * </p>
     * Queer Coded by Striker-909.
     *
     * @param targetX        The x coordinate of the field position the robot will approach.
     * @param targetY        the y coordinate of the field position the robot will approach.
     * @param targetAngle    The angle the robot will approach in degrees.
     * @param posTolerance   A value to determine how precisely the robot must reach the set point. If usePrecision is true, a value between 1.0 and 0.0, if not the number of inches from the target that the robot will have to reach.
     * @param angleTolerance The tolerance for reaching the target angle as a positive double. If this is set to 0, the pid will run indefinitely.
     * @param maxPower       The maximum power the DriveTrain may send to each of its motors. Capped at 1.0.
     * @param usePrecision   If true, the robot will attempt to reach the target position exactly, withing tolerance. If false, the robot will move in the general direction of the target position.
     * @param weight         The weight, as a double between 1.0 and 0.0, that this module will contribute to moving to the next step of auto. A total weight of 1.0 or higher is necessary to move to the next step.
     */
    public DriveTrainAutoModule(double targetX, double targetY, double targetAngle, double posTolerance,
                                       double angleTolerance, double maxPower, boolean usePrecision, double weight) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.posTolerance = posTolerance;
        this.angleTolerance = angleTolerance;
        this.maxPower = maxPower;
        this.usePrecision = usePrecision;
        this.weight = weight;
    }

    public double targetX() { return targetX; }
    public double targetY() { return targetY; }
    public double targetAngle() { return targetAngle; }
    public double posTolerance() { return posTolerance; }
    public double angleTolerance() { return angleTolerance; }
    public double maxPower() { return maxPower; }
    public boolean usePrecision() { return usePrecision; }
    public double weight() { return weight; }

    public Pose targetPose() {
        return new Pose(new Vector(targetX, targetY), Angle.degree(targetAngle));
    }
}