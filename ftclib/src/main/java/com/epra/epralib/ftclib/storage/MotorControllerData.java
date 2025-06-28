package com.epra.epralib.ftclib.storage;

/**A record that stores data from a MotorController.
 *<p></p>
 * Queer Coded by Striker-909.*/
public class MotorControllerData {

    long time;
    double power;
    int position;
    int target;
    double velocity;
    double targetVelocity;
    double pidTOutput;
    double pidVOutput;

    public MotorControllerData (long time, double power, int position, int target, double velocity, double targetVelocity, double pidTOutput, double pidVOutput) {
        this.time = time;
        this.power = power;
        this.position = position;
        this.target = target;
        this.velocity = velocity;
        this.targetVelocity = targetVelocity;
        this.pidTOutput = pidTOutput;
        this.pidVOutput = pidVOutput;
    }
}