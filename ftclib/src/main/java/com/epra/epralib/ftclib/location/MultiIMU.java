package com.epra.epralib.ftclib.location;

import android.annotation.SuppressLint;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.storage.logdata.IMUData;
import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
/**
 * Increases the functionality of the IMU class and combines data from multiple IMUs.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class MultiIMU {

    private Angle baseYaw = new Angle();
    private Angle basePitch = new Angle();
    private Angle baseRoll = new Angle();

    private final FileWriter logWriter;
    private final Gson gson;

    public enum AXIS {
        YAW,
        PITCH,
        ROLL
    }

    ArrayList<IMU> imus = new ArrayList<IMU>();

    /**
     * Increases the functionality of the IMU class and combines data from multiple IMUs.
     * <p></p>
     * Expands the functionality of one IMU.
     * @param imu An IMU.
     * @param imus Any number of additional IMUs.
     */
    public MultiIMU(IMU imu, IMU... imus) throws IOException {
        this.imus.add(imu);
        this.imus.addAll(java.util.Arrays.asList(imus));

        gson = new Gson();

        @SuppressLint("SimpleDateFormat") SimpleDateFormat ft = new SimpleDateFormat("ddMMyyyy:HH:mm");
        File logJson = AppUtil.getInstance().getSettingsFile("logs/IMU_log_" + ft.format(new Date()) + ".json");
        logWriter = new FileWriter(logJson, true);
        logWriter.write("[");
    }

    /**
     * @return The average yaw angle.
     */
    public Angle getYaw() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = Angle.degree(imus.get(i).getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
        return Geometry.subtract(Geometry.average(angle), baseYaw);
    }

    /**
     * @return The average pitch angle.
     */
    public Angle getPitch() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = Angle.degree(imus.get(i).getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        }
        return Geometry.subtract(Geometry.average(angle), basePitch);
    }

    /**
     * @return The average roll angle.
     */
    public Angle getRoll() {
        Angle[] angle = new Angle[imus.size()];
        for (int i = 0; i < imus.size(); i++) {
            angle[i] = Angle.degree(imus.get(i).getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
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
            yaw[i] = Angle.degree(imus.get(i).getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            pitch[i] = Angle.degree(imus.get(i).getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            roll[i] = Angle.degree(imus.get(i).getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        }
        baseYaw = Geometry.average(yaw);
        basePitch = Geometry.average(pitch);
        baseRoll = Geometry.average(roll);
    }

    /**Saves IMU data to internal logs. Also saves log data to a JSON file on the robot for post-match analysis.
     * @return A IMUData record with data from this log.*/
    public IMUData log() throws IOException {
        IMUData data = new IMUData(getYaw().degree(), getPitch().degree(), getRoll().degree());
        logWriter.write("\n" + gson.toJson(data) + ",");
        return data;
    }

    /**Closes the JSON file that this IMUExpanded is writing to.*/
    public void closeLog() throws IOException {
        logWriter.write("]");
        logWriter.close();
    }
}