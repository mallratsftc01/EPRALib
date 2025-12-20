package org.firstinspires.ftc.examples;

import com.epra.epralib.ftclib.location.MultiIMU;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.movement.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.movement.PIDController;
import com.epra.epralib.ftclib.storage.autonomous.AutoProgram;
import com.epra.epralib.ftclib.storage.autonomous.AutoStep;
import com.epra.epralib.ftclib.storage.autonomous.MotorControllerAutoModule;
import com.epra.epralib.ftclib.movement.PIDGains;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

@Autonomous
public class AutoExample extends LinearOpMode {

    //These variables lead to the JSON files that control the vast majority of auto
    private final String AUTO_DIRECTORY = "auto";
    private final String PID_SETTINGS_FILENAME = "pid.json";
    private final String ENCODER_SETTINGS_FILENAME = "encoder.json";
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

    private AutoProgram program;

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
                .useEncoderSettingsFile(ENCODER_SETTINGS_FILENAME)
                .heading(imu::getYaw)
                .startPose(new Pose(new Vector(0, 0), Angle.degree(0)))
                .loggingTargets(Odometry.LoggingTarget.X, Odometry.LoggingTarget.Y)
                .build();

        //Initializing the DriveTrain
        drive = new DriveTrain.Builder()
                .motor(frontRight)
                .motor(frontLeft)
                .motor(backRight)
                .motor(frontLeft)
                .driveType(DriveTrain.DriveType.MECANUM)
                .build();

        //Setting up the MotorControllers that are not part of the DriveTrain
        nonDriveMotors = new HashMap<>();
        //Add MotorControllers like so:
        /* nonDriveMotors.put("ID",
        new MotorController.Builder(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "motorController1")))
                .id("ID")
                .addLogTarget(MotorController.LogTarget.POSITION)
                .build());*/

        PIDController.getPIDsFromFile(PID_SETTINGS_FILENAME);

        // Setting up the AutoProgram
        HashMap<String, Supplier<Double>> dataSuppliers = new HashMap<>();
        dataSuppliers.put("Time.Seconds", () -> (double)System.currentTimeMillis() / 1000.0);
        dataSuppliers.put("Position.X", () -> odometry.getPose().pos.x());
        dataSuppliers.put("Position.Y", () -> odometry.getPose().pos.y());
        dataSuppliers.put("Position.Theta", () -> imu.getYaw().degree());
        dataSuppliers.put("Velocity.X", () -> odometry.getVelocity().x());
        dataSuppliers.put("Velocity.Y", () -> odometry.getVelocity().y());
        dataSuppliers.put("Acceleration.X", () -> odometry.getAcceleration().x());
        dataSuppliers.put("Acceleration.Y", () -> odometry.getAcceleration().y());
        for (String key : nonDriveMotors.keySet()) {
            MotorController motor = nonDriveMotors.get(key);
            dataSuppliers.put(key + ".Position", () -> motor.getCurrentPosition());
            dataSuppliers.put(key + ".Velocity", () -> motor.getVelocity());
        }

        program = new AutoProgram(AUTO_DIRECTORY, dataSuppliers);

        LogController.logInfo("Waiting for start...");
        waitForStart();
        LogController.logInfo("Starting Autonomous.");
        long startTime = System.currentTimeMillis();
        long saveTime = startTime;
        while (program.autoActive()) {
            //Logs
            LogController.logData();

            //Updates all active PID loops
            PIDController.update();

            //Updates the auto program
            program.updateStep();
            AutoStep currentStep = program.getCurrentStep();

            double weight = 0.0;

            //Updates the DriveTrain with new instructions
            drive.posPIDMecanumDrive(currentStep.driveTrainModule());

            //Updates all the MotorControllers with new instructions
            if (currentStep.motorControllerModules() != null) {
                for (MotorControllerAutoModule m : currentStep.motorControllerModules()) {
                    if (m.tolerance() == -1.0) {
                        nonDriveMotors.get(m.id()).setPower(m.power());
                    } else {
                        nonDriveMotors.get(m.id()).moveToTarget(m);
                    }
                }
            }
        }
    }
}