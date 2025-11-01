package com.epra.epralib.ftclib.storage.logdata;

/**A record that stores data from a MotorController.
 *<p></p>
 * Queer Coded by Striker-909.*/
public record MotorControllerData(long time, double power, int position, int target, double velocity, double targetVelocity, double pidTOutput, double pidVOutput) {}