package com.epra.epralib.ftclib.storage;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUStorage{
    YawPitchRollAngles[] orientation;

    /**Stores values for the IMU and uses them to perform calculations.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
    public IMUStorage (YawPitchRollAngles[] orientationIn) {
        updateIMUValues(orientationIn);}

    /**Stores values provided.*/
    public void updateIMUValues(YawPitchRollAngles[] orientationIn) {
        orientation = new YawPitchRollAngles[orientationIn.length];
        for (int ii = 0; ii < orientation.length; ii++) {orientation[ii] = orientationIn[ii];}
    }
    /**Returns the average orientation of the IMU(s) based on stored values.*/
    public double avgIMU(int axis, AngleUnit angleUnit) {
        double r = 0;
        for (int ii = 0; ii < orientation.length; ii++) {
            switch (axis) {
                case 0:
                    r += orientation[ii].getYaw(angleUnit);
                    break;
                case 1:
                    r += orientation[ii].getPitch(angleUnit);
                    break;
                case 2:
                    r += orientation[ii].getRoll(angleUnit);
                    break;
            }
        }
        return r / orientation.length;
    }
    /**Returns the distance between the current orientation of the IMU(s) and the target based on stored values. Do not use, always use trueDistIMU.*/
    public double distIMU(int axis, AngleUnit angleUnit, double target) {return target - avgIMU(axis, angleUnit);}
    /**Returns the true distance between the orientation of the IMU(s) and the target based on stored values, including looping from 360 to 1.*/
    public double trueDistIMU(int axis, AngleUnit angleUnit, double target) {
        double current = avgIMU(axis, angleUnit);
        double newTarget = target;
        if (Math.min(target, current) == target) {newTarget += 360;}
        else {current += 360;}
        double dist1 = Math.abs(distIMU(axis,angleUnit, target));
        double r =(Math.min(dist1, Math.abs((newTarget - current))) == dist1) ? distIMU(axis,angleUnit, target) : (newTarget - current);
        return (Math.abs(r) - 180) * Math.signum(r) * -1.0;
    }
}
