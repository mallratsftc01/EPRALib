package storage;

import location.IMUExpanded;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class SensorStorageMaster {
    public final int MOTOR_FREQUENCY = 10;
    private int mTracker = 0;
    public final int BINARY_SENSOR_FREQUENCY = 20;
    private int bsTracker = 0;
    public final int IMU_STORAGE_FREQUENCY = 50;
    private int imuTracker = 0;
    private DcMotorEx[] motors;
    private TouchSensor[] touchSensors;
    private IMUExpanded imu;
    public SensorIntStorage motorPositions;
    public SensorDoubleStorage motorVelocities;
    public SensorBooleanStorage binarySensors;
    public IMUStorage imuStorage;
    /**Stores values from sensors and coordinates their updating.
    *<p></p>
    *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
    public SensorStorageMaster(DcMotorEx[] motorsIn, TouchSensor[] touchSensorsIn, IMUExpanded imuIn) {
        motors = new DcMotorEx[motorsIn.length];
        for (int ii = 0; ii < motorsIn.length; ii++) {motors[ii] = motorsIn[ii];}
        touchSensors = new TouchSensor[touchSensorsIn.length];
        for (int ii = 0; ii < touchSensorsIn.length; ii++) {touchSensors[ii] = touchSensorsIn[ii];}
        imu = imuIn;

        int[] p = new int[motors.length];
        double[] v = new double[motors.length];
        for (int ii = 0; ii < motors.length; ii++) {
            p[ii] = motors[ii].getCurrentPosition();
            v[ii] = motors[ii].getVelocity();
        }
        motorPositions = new SensorIntStorage(p);
        motorVelocities = new SensorDoubleStorage(v);

        boolean[] b = new boolean[touchSensors.length];
        for (int ii = 0; ii < touchSensors.length; ii++) {b[ii] = touchSensors[ii].isPressed();}
        binarySensors = new SensorBooleanStorage(b);

        //imuStorage = new IMUStorage(imu.getOrientation());
    }
    /**Updates the values associated with the motors.*/
    public void updateMotors() {
        int[] p = new int[motors.length];
        double[] v = new double[motors.length];
        for (int ii = 0; ii < motors.length; ii++) {
            p[ii] = motors[ii].getCurrentPosition();
            v[ii] = motors[ii].getVelocity();
        }
        motorPositions.setSensorValues(p);
        motorVelocities.setSensorValues(v);
    }
    /**Updates the values associated with the binary sensors.*/
    public void updateBinarySensors() {
        boolean[] b = new boolean[touchSensors.length];
        for (int ii = 0; ii < touchSensors.length; ii++) {b[ii] = touchSensors[ii].isPressed();}
        binarySensors.setSensorValues(b);
    }
    /**Updates the values associated with the IMU.*/
    public void updateIMU() {
        //imuStorage.updateIMUValues(imu.getOrientation());
    }
    /**Updates all values at their frequency.*/
    public void update() {
        if (++mTracker >= MOTOR_FREQUENCY) {
            mTracker = 0;
            updateMotors();
        }
        if (++bsTracker >= BINARY_SENSOR_FREQUENCY) {
            bsTracker = 0;
            updateBinarySensors();
        }
        if (++imuTracker >= IMU_STORAGE_FREQUENCY) {
            imuTracker = 0;
            updateIMU();
        }
    }
}
