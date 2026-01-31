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
///         "driveMode": string,
///         "maxPower": float/string,
///         "x": float/string,
///         "y": float/string,
///         "angle": float/string,
///         "posTolerance": float/string,
///         "angleTolerance": float/string
///     }
/// </code></pre>
///
/// Intended to be used as a part of an [AutoStep].
///
/// A [DriveTrain] auto module can use one of four different `driveMode`s:
///
/// - `none` will halt the robot, not moving at all.
///
/// - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
/// treated as a [Vector] relative to the field.
/// It will also rotate to the specified `angle` within the `angleTolerance`
/// (1 being no precision and 0 being infinite precision), relative to the field.
/// This drive mode uses [DriveTrain#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)].
///
/// - `precise_targeted_drive` drives the robot to the [Pose] on the field specified by `x`, `y`, and the `angle`.
/// Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision and 0 being infinite precision)
/// for both. This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double, boolean)].
///
/// - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified
/// by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that
/// is considered "close enough". `angle` is handled the same way as the other drive modes.
/// This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double)].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see DriveTrain#useDriveTrainAutoModule(DriveTrainAutoModule)
public class DriveTrainAutoModule {

    private final String driveMode;
    private final AutoProgram.AutoModuleDataSupplier x;
    private final AutoProgram.AutoModuleDataSupplier y;
    private final AutoProgram.AutoModuleDataSupplier angle;
    private final AutoProgram.AutoModuleDataSupplier posTolerance;
    private final AutoProgram.AutoModuleDataSupplier angleTolerance;
    private final AutoProgram.AutoModuleDataSupplier maxPower;

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
    /// A [DriveTrain] auto module can use one of four different `driveMode`s:
    ///
    /// - `none` will halt the robot, not moving at all.
    ///
    /// - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
    /// treated as a [Vector] relative to the field.
    /// It will also rotate to the specified `angle` within the `angleTolerance`
    /// (1 being no precision and 0 being infinite precision), relative to the field.
    /// This drive mode uses [DriveTrain#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)].
    ///
    /// - `precise_targeted_drive` drives the robot to the [Pose] on the field specified by `x`, `y`, and the `angle`.
    /// Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision and 0 being infinite precision)
    /// for both. This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double, boolean)].
    ///
    /// - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified
    /// by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that
    /// is considered "close enough". `angle` is handled the same way as the other drive modes.
    /// This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double)].
    ///
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
        this.maxPower = new AutoProgram.AutoModuleDataSupplierNumber(maxPower);
        this.x = new AutoProgram.AutoModuleDataSupplierNumber(x);
        this.y = new AutoProgram.AutoModuleDataSupplierNumber(y);
        this.angle = new AutoProgram.AutoModuleDataSupplierNumber(angle);
        this.posTolerance = new AutoProgram.AutoModuleDataSupplierNumber(posTolerance);
        this.angleTolerance = new AutoProgram.AutoModuleDataSupplierNumber(angleTolerance);
    }

    /// A class that stores instructions for the [DriveTrain] during auto.
    ///
    /// JSON Format:
    /// <pre><code>
    ///     {
    ///         "driveMode": string,
    ///         "maxPower": float/string,
    ///         "x": float/string,
    ///         "y": float/string,
    ///         "angle": float/string,
    ///         "posTolerance": float/string,
    ///         "angleTolerance": float/string
    ///     }
    /// </code></pre>
    ///
    /// Intended to be used as a part of an [AutoStep].
    ///
    /// Data can be strings that can reference [AutoProgram] data suppliers and are parsed using
    /// [AutoProgram#parseArithmatic(String)].
    ///
    /// A [DriveTrain] auto module can use one of four different `driveMode`s:
    ///
    /// - `none` will halt the robot, not moving at all.
    ///
    /// - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
    /// treated as a [Vector] relative to the field.
    /// It will also rotate to the specified `angle` within the `angleTolerance`
    /// (1 being no precision, and 0 being infinite precision), relative to the field.
    /// This drive mode uses [DriveTrain#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)].
    ///
    /// - `precise_targeted_drive` drives the robot to the [Pose] on the field specified by `x`, `y`, and the `angle`.
    /// Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision, and 0 being infinite precision)
    /// for both. This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double, boolean)].
    ///
    /// - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified
    /// by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that
    /// is considered "close enough". `angle` is handled the same way as the other drive modes.
    /// This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double)].
    ///
    /// @param driveMode The drive mode this auto module will use
    /// @param maxPower       The maximum absolute power the drivetrain can send to each of its motors, capped at 1
    /// @param x        The `x` value as specified by the `driveMode`
    /// @param y        The `y` value as specified by the `driveMode`
    /// @param angle    The target angle of the robot in degrees
    /// @param posTolerance   A value to determine how precisely the robot must reach the target,
    /// if the driveMode instructs the robot to move towards a target position
    /// @param angleTolerance The tolerance for reaching the `angle` (1 being no precision and 0 being infinite precision)
    /// @see DriveTrain#useDriveTrainAutoModule(DriveTrainAutoModule)
    public DriveTrainAutoModule(String driveMode, AutoProgram.AutoModuleDataSupplier maxPower, AutoProgram.AutoModuleDataSupplier x,
                                AutoProgram.AutoModuleDataSupplier y, AutoProgram.AutoModuleDataSupplier angle,
                                AutoProgram.AutoModuleDataSupplier posTolerance, AutoProgram.AutoModuleDataSupplier angleTolerance) {
        this.driveMode = driveMode;
        this.maxPower = maxPower;
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.posTolerance = posTolerance;
        this.angleTolerance = angleTolerance;
    }

    /// Returns the drive mode of this [DriveTrain] auto module.
    ///
    /// A [DriveTrain] auto module can use one of four different `driveMode`s:
    ///
    /// - `none` will halt the robot, not moving at all.
    ///
    /// - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
    /// treated as a [Vector] relative to the field.
    /// It will also rotate to the specified `angle` within the `angleTolerance`
    /// (1 being no precision and 0 being infinite precision), relative to the field.
    /// This drive mode uses [DriveTrain#fieldOrientedMecanumDrive(Vector, Vector, double, boolean)].
    ///
    /// - `precise_targeted_drive` drives the robot to the [Pose] on the field specified by `x`, `y`, and the `angle`.
    /// Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision and 0 being infinite precision)
    /// for both. This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double, boolean)].
    ///
    /// - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified
    /// by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that
    /// is considered "close enough". `angle` is handled the same way as the other drive modes.
    /// This drive mode uses [DriveTrain#posPIDMecanumDrive(double, double, double)].
    /// @return The drive mode
    public String driveMode() { return driveMode; }

    /// Returns the `x` value of this [DriveTrain] auto module,
    /// as specified by the [DriveTrainAutoModule#driveMode()].
    /// @return The `x` value
    public double x() { return x.get(); }
    /// Returns the `y` value of this [DriveTrain] auto module,
    /// as specified by the [DriveTrainAutoModule#driveMode()].
    /// @return The `y` value
    public double y() { return y.get(); }
    /// Returns the target angle for the [DriveTrain].
    /// @return The target angle
    public double angle() { return angle.get(); }
    /// Returns the tolerance for reaching the target position.
    /// @return The positional tolerance
    public double posTolerance() { return posTolerance.get(); }
    /// Returns the tolerance for reaching the target angle.
    /// @return The angular tolerance
    public double angleTolerance() { return angleTolerance.get(); }
    /// The maximum absolute power the [DriveTrain] can send to each of its motors.
    /// @return The maximum absolute power the drivetrain can send to each of its motors
    public double maxPower() { return maxPower.get(); }

    /// The target [Pose] for the [DriveTrain].
    /// @return The target pose
    public Pose targetPose() {
        return new Pose(new Vector(x.get(), y.get()), Angle.degree(angle.get()));
    }

    /// Returns the drive train auto module with its target pose reflected over the x-axis.
    /// @return The drive train auto module reflected over the x-axis
    public DriveTrainAutoModule reflectX() {
        return new DriveTrainAutoModule(driveMode, maxPower.get(), -x.get(), y.get(), (180 - angle.get()) % 360, posTolerance.get(), angleTolerance.get());
    }
    /// Returns the drive train auto module with its target pose reflected over the y-axis.
    /// @return The drive train auto module reflected over the y-axis
    public DriveTrainAutoModule reflectY() {
        return new DriveTrainAutoModule(driveMode, maxPower.get(), x.get(), -y.get(), (-1 * angle.get()) % 360, posTolerance.get(), angleTolerance.get());
    }

    public boolean equals(DriveTrainAutoModule other) {
        return x.equals(other.x) &&
                y.equals(other.y) &&
                angle.equals(other.angle) &&
                posTolerance.equals(other.posTolerance) &&
                angleTolerance.equals(other.angleTolerance) &&
                maxPower.equals(other.maxPower) &&
                driveMode.equals(other.driveMode);
    }
}