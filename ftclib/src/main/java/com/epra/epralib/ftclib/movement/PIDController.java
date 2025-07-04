package com.epra.epralib.ftclib.movement;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Point;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.storage.PIDGains;

/**Handles the processes of a PID loop.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class PIDController {

    private double k_p, k_i, k_d;

    private double p,i,d;

    private long saveTime;
    private double saveError;
    /**Handles the processes of a PID loop.
     * @param k_p The proportional gain.
     * @param k_i The integral gain.
     * @param k_d The derivative gain.*/
    public PIDController(double k_p, double k_i, double k_d) {
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
        reset();
    }

    /**Handles the processes of a PID loop.
     * @param pidGains A PIDGains record containing the gains for this PIDController.*/
    public PIDController(PIDGains pidGains) { this(pidGains.kp(), pidGains.ki(), pidGains.kd()); }

    /**Sets the proportional gain to a certain value.
     * @param k_p The proportional gain. */
    public void tuneP(double k_p) { this.k_p = k_p; }
    /**Sets the integral gain to a certain value.
     * @param k_i The integral gain. */
    public void tuneI(double k_i) { this.k_i = k_i; }
    /**Sets the derivative gain to a certain value.
     * @param k_d The derivative gain. */
    public void tuneD(double k_d) { this.k_d = k_d; }
    /**Sets the gains to certain values.
     * @param k_p The proportional gain.
     * @param k_i The integral gain.
     * @param k_d The derivative gain.*/
    public void tune(double k_p, double k_i, double k_d) {
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
    }

    /**Sets the gains to certain values.
     * @param pidGains A PIDGains record containing the gains for this PIDController.
     */
    public void tune(PIDGains pidGains) { this.tune(pidGains.kp(), pidGains.ki(), pidGains.kd()); }

    /**Runs one instance of a PID loop.
     * @param current The current position.
     * @param target The targeted position.
     * @return The outputted power of the PID loop.*/
    public double runPID(double current, double target) {
        double currentError = target - current;
        if (saveError == 0) { saveError = currentError; }
        long currentTime = System.currentTimeMillis();

        p = k_p * currentError;
        i += k_i * (currentError * (currentTime - saveTime));
        if (Math.abs(i) > 1) { i = Math.signum(i); }
        d = k_d * (currentError - saveError) / (currentTime - saveTime);

        saveError = currentError;
        saveTime = currentTime;

        return p + i + d;
    }

    /**Runs one instance of a PID loop for an angle. This can be useful for turning the robot to a target angle.
     * @param current The current angle.
     * @param target The targeted angle.
     * @return The outputted power of the PID loop.*/
    public double runPIDAngle(Angle current, Angle target) {
        double currentError = Geometry.subtract(target, current).getRadian();
        currentError -= (currentError > Math.PI) ? Math.PI * 2: 0;
        currentError *= -1.0;
        if (saveError == 0) { saveError = currentError; }
        long currentTime = System.currentTimeMillis();

        p = k_p * currentError;
        i += k_i * (currentError * (currentTime - saveTime));
        if (Math.abs(i) > 1) { i = Math.signum(i); }
        d = k_d * (currentError - saveError) / (currentTime - saveTime);

        saveError = currentError;
        saveTime = currentTime;

        return p + i + d;
    }

    /**Runs one instance of a PID loop for a point. This can be useful for moving the robot to a target point.
     * @param current The current position.
     * @param target The targeted position.
     * @return A vector of the outputted power and angle of the PID loop.*/
    public Vector runPIDPoint(Point current, Point target) {
        double currentError = Geometry.pythagorean(target, current);
        Angle angle = Geometry.atan(new Point(-1 * (target.y - current.y), (target.x - current.x)));
        if (saveError == 0) { saveError = currentError; }
        long currentTime = System.currentTimeMillis();

        p = k_p * currentError;
        i += k_i * (currentError * (currentTime - saveTime));
        if (Math.abs(i) > 1) { i = Math.signum(i); }
        d = k_d * (currentError - saveError) / (currentTime - saveTime);

        saveError = currentError;
        saveTime = currentTime;

        return new Vector(p + i + d, angle);
    }

    /**Resets the PID loop and all the gains to 0.*/
    public void reset() {
        p = 0;
        i = 0;
        d = 0;
        saveTime = System.currentTimeMillis();
        saveError = 0;
    }
}
