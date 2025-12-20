package com.epra.epralib.ftclib.storage.autonomous;

import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.DriveTrain;

/// A class that stores instructions for the [DriveTrain] during auto.
/// <p>
/// JSON Format:
/// <pre><code>
///     {
///         "targetX": float,
///         "targetY": float,
///         "targetAngle": float,
///         "posTolerance": float,
///         "angleTolerance": float,
///         "maxPower": float,
///         "usePrecision": boolean,
///     } </code></pre>
/// </p>
///
/// Intended to be used as a part of an [AutoStep].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see DriveTrain#posPIDMecanumDrive(DriveTrainAutoModule)
public class DriveTrainAutoModule {

    private final double targetX;
    private final double targetY;
    private final double targetAngle;
    private final double posTolerance;
    private final double angleTolerance;
    private final double maxPower;
    private final boolean usePrecision;

    /// A class that stores instructions for the [DriveTrain] during auto.
    ///
    /// JSON Format:
    /// <pre><code>
    ///     {
    ///         "targetX": float,
    ///         "targetY": float,
    ///         "targetAngle": float,
    ///         "posTolerance": float,
    ///         "angleTolerance": float,
    ///         "maxPower": float,
    ///         "usePrecision": boolean,
    ///     } </code></pre>
    ///
    /// Intended to be used as a part of an [AutoStep].
    /// @param targetX        The `x` coordinate of the field position the robot will approach
    /// @param targetY        the `y` coordinate of the field position the robot will approach
    /// @param targetAngle    The angle the robot will approach in degrees
    /// @param posTolerance   A value to determine how precisely the robot must reach the set point.
    /// If `usePrecision` is true, a value between 1.0 and 0.0,
    /// if not the number of inches from the target that the robot will have to reach
    /// @param angleTolerance The tolerance for reaching the target angle as a positive double.
    /// If this is set to 0, the pid will run indefinitely
    /// @param maxPower       The maximum absolute power the drivetrain can send to each of its motors. Capped at 1.0
    /// @param usePrecision   If true, the robot will attempt to reach the target position exactly, withing tolerance.
    /// If false, the robot will move in the general direction of the target position
    /// @see DriveTrain#posPIDMecanumDrive(DriveTrainAutoModule)
    public DriveTrainAutoModule(double targetX, double targetY, double targetAngle, double posTolerance,
                                       double angleTolerance, double maxPower, boolean usePrecision) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetAngle = targetAngle;
        this.posTolerance = posTolerance;
        this.angleTolerance = angleTolerance;
        this.maxPower = maxPower;
        this.usePrecision = usePrecision;
    }

    /// Returns the target `x` position for the [DriveTrain].
    /// @return The target `x` position
    public double targetX() { return targetX; }
    /// Returns the target `y` position for the [DriveTrain].
    /// @return The target `y` position
    public double targetY() { return targetY; }
    /// Returns the target angle for the [DriveTrain].
    /// @return The target angle
    public double targetAngle() { return targetAngle; }
    /// Returns the tolerance for reaching the target position.
    /// @return The positional tolerance
    public double posTolerance() { return posTolerance; }
    /// Returns the tolerance for reaching the target angle.
    /// @return The angular tolerance
    public double angleTolerance() { return angleTolerance; }
    /// The maximum absolute power the [DriveTrain] can send to each of its motors.
    /// @return The maximum absolute power the drivetrain can send to each of its motors
    public double maxPower() { return maxPower; }
    /// If true, the robot will attempt to reach the target position exactly, withing tolerance.
    /// If false, the robot will move in the general direction of the target position.
    /// @return If the drivetrain can use precision for this motion
    public boolean usePrecision() { return usePrecision; }

    /// The target [Pose] for the [DriveTrain].
    /// @return The target pose
    public Pose targetPose() {
        return new Pose(new Vector(targetX, targetY), Angle.degree(targetAngle));
    }
}