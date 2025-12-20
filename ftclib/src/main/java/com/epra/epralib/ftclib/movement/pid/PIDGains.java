package com.epra.epralib.ftclib.movement.pid;

/// A record that stores gains for a [PIDController].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
///
/// @param kp The `p` constant
/// @param ki The `i` constant
/// @param kd The `d` constant
public record PIDGains(String id, double kp, double ki, double kd) {}