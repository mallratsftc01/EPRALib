package com.epra.epralib.ftclib.storage;

import java.util.List;

/**A record that stores instructions one step of auto.
 *<p></p>
 * Queer Coded by Striker-909.
 * @param time How long this step should take, in milliseconds.
 * @param timeWeight The weight, as a double between 1.0 and 0.0, that hitting the allotted amount of time will contribute to moving to the next step of auto. A total weight of 1.0 or higher is necessary to move to the next step.
 * @param driveTrainModule A module to control the DriveTrain.
 * @param motorControllerModules A List of modules to control MotorControllers that are not part of the DriveTrain.
 * @param crServoModules A List of modules to control CRServos.
 * @param servoModules A List of modules to control Servos.
 * */
public record AutoStep(long time, double timeWeight, DriveTrainAutoModule driveTrainModule, List<MotorControllerAutoModule> motorControllerModules, List<CRServoAutoModule> crServoModules, List<ServoAutoModule> servoModules) {}