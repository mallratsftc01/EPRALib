package com.epra.epralib.ftclib.movement.pid;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.function.Supplier;

/// A storage class for the data necessary for [PIDController] run a PID loop.
///
/// A PID loop uses Proportional, Integral, and Derivative elements of feedback to correct error smoothly.
/// [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class PIDData {

    public double kp, ki, kd;
    public double i;
    public double saveError;
    public double output;
    private final Supplier<Double> errorSupplier;

    /// A storage class for the data necessary for [PIDController] run a PID loop.
    ///
    /// A PID loop uses Proportional, Integral, and Derivative elements of feedback to correct error smoothly.
    /// [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    /// @param errorSupplier A function that supplies the current error of the system that this PID loop will monitor
    public PIDData(double kp, double ki, double kd, Supplier<Double> errorSupplier) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.errorSupplier = errorSupplier;
        this.saveError = 0;
        this.i = 0;
        this.output = 0;
    }

    /// A storage class for the data necessary for [PIDController] run a PID loop.
    ///
    /// A PID loop uses Proportional, Integral, and Derivative elements of feedback to correct error smoothly.
    /// [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
    /// @param pidCoefficients An object with instructions to tune the PID loop
    /// @param errorSupplier A function that supplies the current error of the system that this PID loop will monitor
    public PIDData(PIDCoefficients pidCoefficients, Supplier<Double> errorSupplier) {
        this(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, errorSupplier);
    }

    /// Retrieves the output from the error supplier.
    /// @return The error supplier's output
    public double getError() { return errorSupplier.get(); }

    /// Modifies the gain constants for this PID loop.
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    public void tune(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    /// Modifies the gain constants for this PID loop
    /// @param pidCoefficients An object with instructions to tune the PID loop
    public void tune(PIDCoefficients pidCoefficients) {
        this.kp = pidCoefficients.p;
        this.ki = pidCoefficients.i;
        this.kd = pidCoefficients.d;
    }
}
