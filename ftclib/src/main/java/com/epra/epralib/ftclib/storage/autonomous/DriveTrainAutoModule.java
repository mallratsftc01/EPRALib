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
/// @see DriveTrain#useDriveTrainAutoModule(DriveTrainAutoModule)
public class DriveTrainAutoModule {

    private final String driveMode;
    private final double x;
    private final double y;
    private final double angle;
    private final double posTolerance;
    private final double angleTolerance;
    private final double maxPower;

    /// A class that stores instructions for the [DriveTrain] during auto.
    ///
    /// JSON Format:
    /// <pre><code>
    ///     {
    ///         "driveMode": string,
    ///         "maxPower": float,
    ///         "x": float,
    ///         "y": float,
    ///         "angle": float,
    ///         "posTolerance": float,
    ///         "angleTolerance": float
    ///     }
    /// </code></pre>
    ///
    /// Intended to be used as a part of an [AutoStep].
    ///
    /// A [DriveTrain] auto module can use one of three different `driveMode`s:
    /// - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
    /// treated as a [Vector] relative to the field.
    /// It will also rotate to the specified `angle` within the `angleTolerance`
    /// (1 being no precision and 0 being infinite precision), relative to the field.
    /// This drive mode uses [DriveTrain#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)].
    /// - `precise_targeted_drive` drives the robot to the [Pose] on the field specified by `x`, `y`, and the `angle`.
    /// Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision and 0 being infinite precision)
    /// for both. This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double, boolean)].
    /// - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified
    /// by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that
    /// is considered "close enough". `angle` is handled the same way as the other drive modes.
    /// This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double)].
    /// @param driveMode The drive mode this auto module will use
    /// @param maxPower       The maximum absolute power the drivetrain can send to each of its motors, capped at 1
    /// @param x        The `x` value as specified by the `driveMode`
    /// @param y        The `y` value as specified by the `driveMode`
    /// @param angle    The target angle of the robot in degrees
    /// @param posTolerance   A value to determine how precisely the robot must reach the target,
    /// if the driveMode instructs the robot to move towards a target position
    /// @param angleTolerance The tolerance for reaching the `angle` (1 being no precision and 0 being infinite precision)
    /// @see DriveTrain#useDriveTrainAutoModule(DriveTrainAutoModule)
    public DriveTrainAutoModule(String driveMode, double maxPower, double x, double y, double angle,
                                double posTolerance, double angleTolerance) {
        this.driveMode = driveMode;
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.posTolerance = posTolerance;
        this.angleTolerance = angleTolerance;
        this.maxPower = maxPower;
    }

    /// Returns the drive mode of this [DriveTrain] auto module.
    ///
    /// A [DriveTrain] auto module can use one of three different `driveMode`s:
    /// - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
    /// treated as a [Vector] relative to the field.
    /// It will also rotate to the specified `angle` within the `angleTolerance`
    /// (1 being no precision and 0 being infinite precision), relative to the field.
    /// This drive mode uses [DriveTrain#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)].
    /// - `precise_targeted_drive` drives the robot to the [Pose] on the field specified by `x`, `y`, and the `angle`.
    /// Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision and 0 being infinite precision)
    /// for both. This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double, boolean)].
    /// - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified
    /// by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that
    /// is considered "close enough". `angle` is handled the same way as the other drive modes.
    /// This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double)].
    /// @return The drive mode
    public String driveMode() { return driveMode; }

    /// Returns the `x` value of this [DriveTrain] auto module,
    /// as specified by the [DriveTrainAutoModule#driveMode()].
    /// @return The `x` value
    public double x() { return x; }
    /// Returns the `y` value of this [DriveTrain] auto module,
    /// as specified by the [DriveTrainAutoModule#driveMode()].
    /// @return The `y` value
    public double y() { return y; }
    /// Returns the target angle for the [DriveTrain].
    /// @return The target angle
    public double angle() { return angle; }
    /// Returns the tolerance for reaching the target position.
    /// @return The positional tolerance
    public double posTolerance() { return posTolerance; }
    /// Returns the tolerance for reaching the target angle.
    /// @return The angular tolerance
    public double angleTolerance() { return angleTolerance; }
    /// The maximum absolute power the [DriveTrain] can send to each of its motors.
    /// @return The maximum absolute power the drivetrain can send to each of its motors
    public double maxPower() { return maxPower; }

    /// The target [Pose] for the [DriveTrain].
    /// @return The target pose
    public Pose targetPose() {
        return new Pose(new Vector(x, y), Angle.degree(angle));
    }

    /// Returns the drive train auto module with its target pose reflected over the x-axis.
    /// @return The drive train auto module reflected over the x-axis
    public DriveTrainAutoModule reflectX() {
        return new DriveTrainAutoModule(driveMode, maxPower, -x, y, (180 - angle) % 360, posTolerance, angleTolerance);
    }
    /// Returns the drive train auto module with its target pose reflected over the y-axis.
    /// @return The drive train auto module reflected over the y-axis
    public DriveTrainAutoModule reflectY() {
        return new DriveTrainAutoModule(driveMode, maxPower, x, -y, (-1 * angle) % 360, posTolerance, angleTolerance);
    }

    public boolean equals(DriveTrainAutoModule other) {
        return x == other.x &&
                y == other.y &&
                angle == other.angle &&
                posTolerance == other.posTolerance &&
                angleTolerance == other.angleTolerance &&
                maxPower == other.maxPower &&
                driveMode == other.driveMode;
    }
}