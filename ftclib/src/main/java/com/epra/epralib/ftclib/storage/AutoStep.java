package com.epra.epralib.ftclib.storage;

import java.util.List;

public class AutoStep {
    private final long time;
    private final double timeWeight;
    private final DriveTrainAutoModule driveTrainModule;
    private final List<MotorControllerAutoModule> motorControllerModules;
    private final List<CRServoAutoModule> crServoModules;
    private final List<ServoAutoModule> servoModules;

    /**
     * A record that stores instructions one step of auto.
     * <p>
     * JSON Format:
     * <pre><code>
     *     {
     *       "time": int,
     *       "timeWeight": float,
     *       "driveTrainModule": {
     *           "targetX": float,
     *           "targetY": float,
     *           "targetAngle": float,
     *           "posTolerance": float,
     *           "angleTolerance": float,
     *           "maxPower": float,
     *           "usePrecision": boolean,
     *           "weight": float
     *       }
     *       "motorControllerModules": [
     *           {
     *               "id": string,
     *               "target": int,
     *               "tolerance": float,
     *               "maxPower": float,
     *               "power": float
     *               "weight": float
     *           }
     *       ],
     *       "crServoModules": [
     *           {
     *               "id": string,
     *               "power": float,
     *               "time": int
     *           }
     *       ],
     *       "servoAutoModules": [
     *           {
     *               "id": string,
     *               "targetPosition": float
     *           }
     *       ]
     *     } </code></pre>
     * </p>
     * Queer Coded by Striker-909.
     *
     * @param time                   How long this step should take, in milliseconds.
     * @param timeWeight             The weight, as a double between 1.0 and 0.0, that hitting the allotted amount of time will contribute to moving to the next step of auto. A total weight of 1.0 or higher is necessary to move to the next step.
     * @param driveTrainModule       A module to control the DriveTrain.
     * @param motorControllerModules A List of modules to control MotorControllers that are not part of the DriveTrain.
     * @param crServoModules         A List of modules to control CRServos.
     * @param servoModules           A List of modules to control Servos.
     */
    public AutoStep(long time, double timeWeight,
                    DriveTrainAutoModule driveTrainModule,
                    List<MotorControllerAutoModule> motorControllerModules,
                    List<CRServoAutoModule> crServoModules, List<ServoAutoModule> servoModules) {

        this.time = time;
        this.timeWeight = timeWeight;
        this.driveTrainModule = driveTrainModule;
        this.motorControllerModules = motorControllerModules;
        this.crServoModules = crServoModules;
        this.servoModules = servoModules;
    }

    public long time() { return time; }
    public double timeWeight() { return timeWeight; }
    public DriveTrainAutoModule driveTrainModule() { return driveTrainModule; }
    public List<MotorControllerAutoModule> motorControllerModules() { return motorControllerModules; }
    public List<CRServoAutoModule> crServoModules() { return crServoModules; }
    public List<ServoAutoModule> servoModules() { return servoModules; }
}