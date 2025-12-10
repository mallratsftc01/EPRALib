package org.firstinspires.ftc.examples;

import android.util.Log;
import com.epra.epralib.ftclib.control.JSONReader;
import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.PIDController;
import com.epra.epralib.ftclib.storage.autonomous.AutoStep;
import com.epra.epralib.ftclib.storage.autonomous.MotorControllerAutoModule;
import com.epra.epralib.ftclib.storage.initialization.PIDGains;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Autonomous
public class AutoExample extends LinearOpMode {

    //These variables lead to the JSON files that control the vast majority of auto
    private final String STEP_LIST_FILENAME = "auto/lists/step_list.json";
    private final String FINAL_STEP_FILENAME = "auto/steps/final_step.json";
    private final String PID_SETTINGS_FILENAME = "pid/gains.json";
    //The starting position must also be set
    private final Pose START_POSE = new Pose(new Vector(0, 0), new Angle());

    private MotorController frontRight;
    private MotorController backRight;
    private MotorController backLeft;
    private MotorController frontLeft;
    private DriveTrain drive;

    private HashMap<String, MotorController> nonDriveMotors;
    private HashMap<String, CRServo> crServos;
    private HashMap<String, Servo> servos;

    private MultiIMU imu;

    private Odometry odometry;

    ArrayList<String> filenames;
    ArrayList<AutoStep> steps;
    AutoStep currentStep;

    @Override
    public void runOpMode() {
        //Initializes the LogController
        LogController.init();

        //Setting up the IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
        tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
        imu = new MultiIMU.Builder(tempIMU)
                    .loggingTarget(MultiIMU.Axis.YAW)
                    .build();

        //Setting up the MotorControllers for the DriveTrain
        frontRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_FRONT)
                .build();
        backRight = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")))
                .driveOrientation(DriveTrain.Orientation.RIGHT_BACK)
                .build();
        frontLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_FRONT)
                .build();
        backLeft = new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")))
                .driveOrientation(DriveTrain.Orientation.LEFT_BACK)
                .build();

        //Setting up the Odometry
        odometry = new Odometry.Builder()
                .leftEncoder(frontLeft::getCurrentPosition, 0.01, new Vector(8, 4))
                .rightEncoder(backLeft::getCurrentPosition, 0.01, new Vector(-8, 4))
                .perpendicularEncoder(frontRight::getCurrentPosition, 0.01, new Vector(0, 2))
                .heading(imu::getYaw)
                .startPose(new Pose(new Vector(0, 0), Angle.degree(0)))
                .loggingTargets(Odometry.LoggingTarget.X, Odometry.LoggingTarget.Y)
                .build();

        //Setting up the PID gains for the DriveTrain and MotorControllers
        HashMap<String, PIDGains> pidGains = JSONReader.readPIDGains(PID_SETTINGS_FILENAME);

        //Initializing the DriveTrain
        drive = new DriveTrain.Builder()
                .motor(frontRight)
                .motor(frontLeft)
                .motor(backRight)
                .motor(frontLeft)
                .driveType(DriveTrain.DriveType.MECANUM)
                .anglePIDConstants(pidGains.get("DriveTrain_angle"))
                .pointPIDConstants(pidGains.get("DriveTrain_point"))
                .motionPIDConstants(pidGains.get("DriveTrain_motion"))
                .build();

        //Setting up the MotorControllers that are not part of the DriveTrain
        nonDriveMotors = new HashMap<>();
        //Add MotorControllers like so:
        /* nonDriveMotors.put("ID",
        new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "motorController1")))
                .id("ID")
                .addLogTarget(MotorController.LogTarget.POSITION)
                .build());*/

        for (String id : nonDriveMotors.keySet()) {
            nonDriveMotors.get(id).tuneTargetPID(pidGains.get("MotorController_" + id + "_Target"));
            nonDriveMotors.get(id).tuneVelocityPID(pidGains.get("MotorController_" + id + "_Velocity"));
        }

        //Creates the list of filenames to read step files from
        filenames = new ArrayList<>();
        steps = new ArrayList<>();
        filenames.addAll(Arrays.asList(JSONReader.readAuto(STEP_LIST_FILENAME)));

        LogController.logInfo("Waiting for start...");
        waitForStart();
        LogController.logInfo("Starting Autonomous.");
        long startTime = System.currentTimeMillis();
        long saveTime = startTime;
        while ((!filenames.isEmpty() || !steps.isEmpty())) {
            //Logs
            LogController.logData();
            //Updates all active PID loops
            PIDController.update();

            //If no steps are in the queue, refills the queue from the next step file in the list
            if (steps.isEmpty() && !filenames.isEmpty()) {
                JSONReader.read(filenames.get(0), steps, new TypeToken <List<AutoStep>>() {}.getType());
                filenames.remove(0);
                saveTime = System.currentTimeMillis();
                currentStep = steps.get(0);
            }

            double weight = 0.0;

            //Updates the DriveTrain with new instructions
            if (drive.posPIDMecanumDrive(currentStep.driveTrainModule())) {
                weight += currentStep.driveTrainModule().weight();
            }

            //Updates all the MotorControllers with new instructions
            if (currentStep.motorControllerModules() != null) {
                for (MotorControllerAutoModule m : currentStep.motorControllerModules()) {
                    if (m.tolerance() == -1.0) {
                        nonDriveMotors.get(m.id()).setPower(m.power());
                    } else {
                        if (nonDriveMotors.get(m.id()).moveToTarget(m)) {
                            weight += m.weight();
                        }
                    }
                }
            }

            //Checks if enough time has elapsed
            if (System.currentTimeMillis() - saveTime >= currentStep.time()) { weight += currentStep.timeWeight(); }

            //Checks if the weight is large enough to move on to the next step
            if (weight >= 1.0 && !steps.isEmpty()) {
                LogController.logInfo("Step completed. Moving on to next step.");
                steps.remove(0);
                saveTime = System.currentTimeMillis();
                currentStep = steps.get(0);
            }
        }
        LogController.logInfo("Beginning final step.");
        //Repeats everything from the main loop for the final step file
        JSONReader.read(FINAL_STEP_FILENAME, steps, new TypeToken <List<AutoStep>>() {}.getType());
        saveTime = System.currentTimeMillis();
        currentStep = steps.get(0);

        while (System.currentTimeMillis() - startTime < 30000 && !steps.isEmpty()) {
            LogController.logData();
            //Updates all active PID loops
            PIDController.update();

            double weight = 0.0;

            if (drive.posPIDMecanumDrive(currentStep.driveTrainModule())) {
                weight += currentStep.driveTrainModule().weight();
            }

            for (MotorControllerAutoModule mcam : currentStep.motorControllerModules()) {
                if (nonDriveMotors.get(mcam.id()).moveToTarget(mcam)) {
                    weight += mcam.weight();
                }
            }

            if (System.currentTimeMillis() - saveTime >= currentStep.time()) { weight += currentStep.timeWeight(); }

            if (weight >= 1.0 && !steps.isEmpty()) {
                LogController.logInfo("Step completed. Moving on to next step.");
                steps.remove(0);
                saveTime = System.currentTimeMillis();
                currentStep = steps.get(0);
            }
        }
        LogController.logInfo("Autonomous complete.");
        LogController.closeLogs();
    }
}