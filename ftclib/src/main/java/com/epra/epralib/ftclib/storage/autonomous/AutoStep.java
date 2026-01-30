package com.epra.epralib.ftclib.storage.autonomous;

import java.util.List;
import java.util.function.Supplier;

/// A class that stores instructions for one step of auto.
///
/// JSON Format:
/// <pre><code>
///     {
///       "comment": string,
///       "endCondition": string,
///       "driveTrainModule": {
///           "targetX": float,
///           "targetY": float,
///           "targetAngle": float,
///           "posTolerance": float,
///           "angleTolerance": float,
///           "maxPower": float,
///           "usePrecision": boolean,
///       },
///       "motorControllerModules": [
///           {
///               "id": string,
///               "target": int,
///               "tolerance": float,
///               "maxPower": float,
///               "power": float
///           }
///       ]
///     } </code></pre>
///
/// To be used in a list in movement files.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see AutoProgram
public class AutoStep {
    private final String comment;
    protected final String endCondition;
    private final DriveTrainAutoModule driveTrainModule;
    private final List<MotorControllerAutoModule> motorControllerModules;

    /// A class that stores instructions for one step of auto.
    ///
    /// JSON Format:
    /// ``` json
    ///     {
    ///       "comment": string,
    ///       "endCondition": string,
    ///       "driveTrainModule": {
    ///               "driveMode": string,
    ///               "maxPower": float,
    ///               "x": float,
    ///               "y": float,
    ///               "angle": float,
    ///               "posTolerance": float,
    ///               "angleTolerance": float
    ///       },
    ///       "motorControllerModules": [
    ///           {
    ///               "id": string,
    ///               "target": int,
    ///               "tolerance": float,
    ///               "maxPower": float,
    ///               "power": float
    ///           },
    ///         ...
    ///       ]
    ///     }
    /// ```
    ///
    /// To be used in a list in a movement file.
    ///
    /// @param comment A comment describing the function of this auto step
    /// @param endCondition A conditional reference that controls when this auto step ends
    /// @param driveTrainModule A module to control the DriveTrain
    /// @param motorControllerModules A list of modules to control MotorControllers that are not part of the DriveTrain
    /// @see AutoProgram
    public AutoStep(String comment, String endCondition,
                    DriveTrainAutoModule driveTrainModule,
                    List<MotorControllerAutoModule> motorControllerModules) {

        this.comment = comment;
        this.endCondition = endCondition;
        this.driveTrainModule = driveTrainModule;
        this.motorControllerModules = motorControllerModules;
    }

    /// Returns the comment describing the function of this auto step.
    /// @return The comment associated with this auto step
    public String comment() {
        return comment;
    }
    /// Returns the name of the end condition for this auto step.
    /// @return This auto step's end condition
    public String endCondition() { return endCondition; }

    /// Returns the [DriveTrainAutoModule] associated with this auto step.
    /// @return The drivetrain auto module associated with this auto step
    public DriveTrainAutoModule driveTrainModule() {
        return driveTrainModule;
    }
    /// Returns the [List] of [MotorControllerAutoModule]s associated with this auto step.
    /// @return The list of motor controller auto modules associated with this auto step.
    public List<MotorControllerAutoModule> motorControllerModules() {
        return motorControllerModules;
    }

    /// Returns the auto step with its [DriveTrainAutoModule]'s target pose reflected over the x-axis.
    /// @return The auto step with its drive train auto module reflected over the x-axis
    public AutoStep reflectX() {
        return new AutoStep(comment, endCondition, driveTrainModule.reflectX(), motorControllerModules);
    }
    /// Returns the auto step with its [DriveTrainAutoModule]'s target pose reflected over the y-axis.
    /// @return The auto step with its drive train auto module reflected over the y-axis
    public AutoStep reflectY() {
        return new AutoStep(comment, endCondition, driveTrainModule.reflectY(), motorControllerModules);
    }

    public boolean equals(AutoStep other) {
        return comment.equals(other.comment) &&
                endCondition.equals(other.endCondition) &&
                driveTrainModule.equals(other.driveTrainModule) &&
                motorControllerModules.equals(other.motorControllerModules);
    }
}