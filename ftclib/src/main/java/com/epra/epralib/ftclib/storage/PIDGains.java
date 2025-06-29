package com.epra.epralib.ftclib.storage;

/**A record that stores gains for a PIDController.
 *<p></p>
 * Queer Coded by Striker-909.*/
public record PIDGains(String id, double kp, double ki, double kd) {}