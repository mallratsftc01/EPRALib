package com.epra.epralib.ftclib.storage;

/**A record that stores instructions for a Servo during auto.
 *<p></p>
 * Queer Coded by Striker-909.
 * @param id The string id of the Servo this module contains instructions for.
 * @param targetPosition The target position of the Servo.
 * */
public record ServoAutoModule(String id, double targetPosition) {}