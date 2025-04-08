package epra.location;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Collections;

import epra.math.geometry.Angle;
import epra.math.geometry.Geometry;
/**
 * Increases the functionality of the IMU class.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class IMUExpanded {

    private Angle baseYaw = new Angle(0);
    private Angle basePitch = new Angle(0);
    private Angle baseRoll = new Angle(0);

    public enum AXIS {
        YAW,
        PITCH,
        ROLL
    }

    ArrayList<IMU> imus = new ArrayList<IMU>();

    /**
     * Increases the functionality of the IMU class.
     * <p></p>
     * Expands the functionality of one IMU.
     * @param imu An IMU.
     */
    public IMUExpanded(IMU imu) {
        imus.add(imu);
    }

    /**
     * Increases the functionality of the IMU class.
     * <p></p>
     * Expands the functionality of two IMUs.
     * @param imu1 First IMU.
     * @param imu2 Second IMU.
     */
    public IMUExpanded(IMU imu1, IMU imu2) {
        imus.add(imu1);
        imus.add(imu2);
    }

    /**
     * Increases the functionality of the IMU class.
     * <p></p>
     * Expands the functionality of multiple IMUs.
     * @param imu Array of IMUs.
     */
    public IMUExpanded(IMU[] imu) { Collections.addAll(imus, imu); }

    /**
     * @return The average yaw angle.
     */
    public Angle getYaw() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = new Angle(imus.get(i).getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
        return Geometry.subtract(Geometry.average(angle), baseYaw);
    }

    /**
     * @return The average pitch angle.
     */
    public Angle getPitch() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = new Angle(imus.get(i).getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        }
        return Geometry.subtract(Geometry.average(angle), basePitch);
    }

    /**
     * @return The average roll angle.
     */
    public Angle getRoll() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = new Angle(imus.get(i).getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        }
        return Geometry.subtract(Geometry.average(angle), baseRoll);
    }
    /**@param axis Axis of angle.
     * @return The average angle of that axis.*/
    public Angle get(AXIS axis) {
        return switch (axis) {
            case YAW -> getYaw();
            case PITCH -> getPitch();
            case ROLL -> getRoll();
        };
    }

    /**Sets all the angles to 0 at the current orientation.*/
    public void recenter() {
        Angle[] yaw = new Angle[imus.size()];
        Angle[] pitch = new Angle[imus.size()];
        Angle[] roll = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            yaw[i] = new Angle(imus.get(i).getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            pitch[i] = new Angle(imus.get(i).getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            roll[i] = new Angle(imus.get(i).getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        }
        baseYaw = Geometry.average(yaw);
        basePitch = Geometry.average(pitch);
        baseRoll = Geometry.average(roll);
    }
}