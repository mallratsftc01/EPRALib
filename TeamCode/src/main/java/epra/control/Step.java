package org.firstinspires.ftc.teamcode;

import epra.location.Pose;
import epra.math.geometry.Angle;
import epra.math.geometry.Point;
/**A storage class for a single step of auto.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Step {

    public double x;
    public double y;
    public double theta;
    public double pos_tolerance;
    public double angle_tolerance;
    public double drive_max;

    public double arm_target;
    public double arm_max;
    public double arm_tolerance;

    public double lift_target;
    public double lift_max;
    public double lift_tolerance;

    public boolean claw_open;
    public boolean wrist_down;
    public int bucket_pos;

    public long millis;

    /**Returns the target Pose of this step.
     * @return The pose of this step.*/
    public Pose getPose() { return new Pose(new Point(x,y), new Angle(theta)); }
}
