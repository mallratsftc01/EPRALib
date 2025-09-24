package com.epra.epralib.ftclib.storage;

import java.util.function.Supplier;

/**A class that stores data essential to run a PID loop.
 *<p></p>
 * Queer Coded by Striker-909.*/
public class PIDData {

    public double kp, ki, kd;
    public double i;
    public double saveError;
    public double output;
    private final Supplier<Double> errorSupplier;

    /**A class that stores data essential to run a PID loop.
     @param kp The p gain for the PID loop.
     @param ki The i gain for the PID loop.
     @param kd The d gain for the PID loop.
     @param errorSupplier A supplier function that returns the error in position.*/
    public PIDData(double kp, double ki, double kd, Supplier<Double> errorSupplier) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.errorSupplier = errorSupplier;
        this.saveError = 0;
        this.i = 0;
        this.output = 0;
    }

    /**A class that stores data essential to run a PID loop.
     @param pidGains The gains for the PID loop.
     @param errorSupplier A supplier that returns the error in position.*/
    public PIDData(PIDGains pidGains, Supplier<Double> errorSupplier) {
        this(pidGains.kp(), pidGains.ki(), pidGains.kd(), errorSupplier);
    }

    /**Returns the current error for this PID loop.
     * @return The current error.*/
    public double getError() { return errorSupplier.get(); }

    /**Sets the PID gains to the specified values.
     * @param kp The p gain for the PID loop.
     * @param ki The i gain for the PID loop.
     * @param kd The d gain for the PID loop.*/
    public void tune(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    /**Sets the PID gains to the specified values.
     * @param pidGains The gains for the PID loop.*/
    public void tune(PIDGains pidGains) {
        this.kp = pidGains.kp();
        this.ki = pidGains.ki();
        this.kd = pidGains.kd();
    }
}
