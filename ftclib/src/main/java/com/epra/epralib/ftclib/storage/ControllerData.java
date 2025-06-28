package com.epra.epralib.ftclib.storage;

/**A record that stores data from a Controller.
 *<p></p>
 * Queer Coded by Striker-909.*/
public record ControllerData(boolean a, boolean b, boolean x, boolean y, boolean up, boolean down, boolean left, boolean right, boolean bumperLeft, boolean bumperRight, boolean stickLeft, boolean stickRight, double leftStickX, double rightStickX, double leftStickY, double rightStickY, double leftTrigger, double rightTrigger) {}