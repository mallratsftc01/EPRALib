package org.firstinspires.ftc.examples;

import com.epra.epralib.ftclib.control.JSONReader;
import com.epra.epralib.ftclib.location.IMUExpanded;
import com.epra.epralib.ftclib.location.Odometry;
import com.epra.epralib.ftclib.location.Pose;
import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Point;
import com.epra.epralib.ftclib.movement.DcMotorExFrame;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.storage.AutoStep;
import com.epra.epralib.ftclib.storage.CRServoAutoModule;
import com.epra.epralib.ftclib.storage.MotorControllerAutoModule;
import com.epra.epralib.ftclib.storage.PIDGains;
import com.epra.epralib.ftclib.storage.ServoAutoModule;
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

    //These variables lead to the json files that control the vast majority of auto
    private final String STEP_LIST_FILENAME = "auto/lists/step_list.json";
    private final String FINAL_STEP_FILENAME = "auto/steps/final_step.json";
    private final String PID_SETTINGS_FILENAME = "pid/gains.json";
    //The starting position must also be set
    private final Pose START_POSE = new Pose(new Point(0, 0), new Angle(0));

    private MotorController frontRight;
    private MotorController backRight;
    private MotorController backLeft;
    private MotorController frontLeft;
    private DriveTrain drive;

    private HashMap<String, MotorController> nonDriveMotors;
    private HashMap<String, CRServo> crServos;
    private HashMap<String, Servo> servos;

    private IMUExpanded imu;

    private Odometry odometry;

    ArrayList<String> filenames;
    ArrayList<AutoStep> steps;
    AutoStep currentStep;

    @Override
    public void runOpMode() {
        try {
            //Setting up the MotorControllers for the DriveTrain
            frontRight = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor")), "front_right");
            frontLeft = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor")), "front_left");
            backRight = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor")), "back_right");
            backLeft = new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor")), "back_left");
            //Initializing the DriveTrain
            drive = new DriveTrain(new MotorController[] {frontLeft, frontRight, backLeft, backRight},
                    new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK},
                    DriveTrain.DriveType.MECANUM);

            //Setting up the MotorControllers that are not part of the DriveTrain
            nonDriveMotors = new HashMap<>();
            //Add MotorControllers like so:
            //nonDriveMotors.put("ID", new MotorController(new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "MOTOR_NAME")), "ID"));

            //Setting up the PID gains for the DriveTrain and MotorControllers
            HashMap<String, PIDGains> pidGains = JSONReader.readPIDGains(PID_SETTINGS_FILENAME);

            //Setting up CRServos and Servos is similar to setting up MotorControllers
            crServos = new HashMap<>();
            servos = new HashMap<>();

            drive.tunePointPID(pidGains.get("DriveTrain_point"));
            drive.tuneAnglePID(pidGains.get("DriveTrain_angle"));
            drive.tuneVectorPID(pidGains.get("DriveTrain_vector"));

            for (String id : nonDriveMotors.keySet()) {
                nonDriveMotors.get(id).tuneTargetPID(pidGains.get("MotorController_" + id + "_Target"));
                nonDriveMotors.get(id).tuneVelocityPID(pidGains.get("MotorController_" + id + "_Velocity"));
            }

            //Setting up the IMU
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            IMU tempIMU = hardwareMap.get(IMU.class, "imu 1");
            tempIMU.initialize(new IMU.Parameters(orientationOnRobot));
            imu = new IMUExpanded(tempIMU);

            //Setting up the Odometry
            odometry = new Odometry(frontLeft, backLeft, frontRight,
                    new Point(7.92784216, 3.75),
                    new Point(-8, 3.75),
                    new Point(0, 2.0),
                    imu,
                    START_POSE
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        //Creates the list of filenames to read step files from
        filenames = new ArrayList<>();
        steps = new ArrayList<>();
        filenames.addAll(Arrays.asList(JSONReader.readAuto(STEP_LIST_FILENAME)));

        waitForStart();
        long startTime = System.currentTimeMillis();
        long saveTime = startTime;
        while ((!filenames.isEmpty() || !steps.isEmpty())) {
            //Logs data from all MotorControllers, the imu, and odometry
            try {
                frontRight.log();
                frontLeft.log();
                backRight.log();
                backLeft.log();
                for (MotorController m : nonDriveMotors.values()) {
                    m.log();
                }
                imu.log();
                odometry.estimatePose();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            //If no steps are in the queue, refills the queue from the next step file in the list
            if (steps.isEmpty() && !filenames.isEmpty()) {
                JSONReader.read(filenames.get(0), steps, new TypeToken <List<AutoStep>>() {}.getType());
                filenames.remove(0);
                saveTime = System.currentTimeMillis();
                currentStep = steps.get(0);
            }

            double weight = 0.0;

            //Updates the DriveTrain with new instructions
            if (drive.posPIDMecanumDrive(odometry.getPose(), odometry.getDeltaPose(), currentStep.driveTrainModule())) {
                weight += currentStep.driveTrainModule().weight();
            }

            //Updates all the MotorControllers with new instructions
            for (MotorControllerAutoModule m : currentStep.motorControllerModules()) {
                if (nonDriveMotors.get(m.id()).moveToTarget(m)) {
                    weight += m.weight();
                }
            }

            //Sets powers and checks times for CRServos
            for (CRServoAutoModule c : currentStep.crServoModules()) {
                if (System.currentTimeMillis() - saveTime < c.time()) {
                    crServos.get(c.id()).setPower(c.power());
                } else {
                    crServos.get(c.id()).setPower(0);
                }
            }

            //Moves all servos to their target positions
            for (ServoAutoModule s : currentStep.servoModules()) {
                servos.get(s.id()).setPosition(s.targetPosition());
            }

            //Checks if enough time has elapsed
            if (System.currentTimeMillis() - saveTime >= currentStep.time()) { weight += currentStep.timeWeight(); }

            //Checks if the weight is large enough to move on to the next step
            if (weight >= 1.0 && !steps.isEmpty()) {
                steps.remove(0);
                saveTime = System.currentTimeMillis();
                currentStep = steps.get(0);
            }
        }

        //Repeats everything from the main loop for the final step file
        JSONReader.read(FINAL_STEP_FILENAME, steps, new TypeToken <List<AutoStep>>() {}.getType());
        saveTime = System.currentTimeMillis();
        currentStep = steps.get(0);

        while (System.currentTimeMillis() - startTime < 30000 && !steps.isEmpty()) {
            try {
                frontRight.log();
                frontLeft.log();
                backRight.log();
                backLeft.log();
                imu.log();
                odometry.estimatePose();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            double weight = 0.0;

            if (drive.posPIDMecanumDrive(odometry.getPose(), odometry.getDeltaPose(), currentStep.driveTrainModule())) {
                weight += currentStep.driveTrainModule().weight();
            }

            for (MotorControllerAutoModule mcam : currentStep.motorControllerModules()) {
                if (nonDriveMotors.get(mcam.id()).moveToTarget(mcam)) {
                    weight += mcam.weight();
                }
            }

            for (CRServoAutoModule c : currentStep.crServoModules()) {
                if (System.currentTimeMillis() - saveTime < c.time()) {
                    crServos.get(c.id()).setPower(c.power());
                } else {
                    crServos.get(c.id()).setPower(0);
                }
            }

            for (ServoAutoModule s : currentStep.servoModules()) {
                servos.get(s.id()).setPosition(s.targetPosition());
            }

            if (System.currentTimeMillis() - saveTime >= currentStep.time()) { weight += currentStep.timeWeight(); }

            if (weight >= 1.0 && !steps.isEmpty()) {
                steps.remove(0);
                saveTime = System.currentTimeMillis();
                currentStep = steps.get(0);
            }
        }

        //Closes all logs
        try {
            frontRight.closeLog();
            frontLeft.closeLog();
            backRight.closeLog();
            backLeft.closeLog();
            for (MotorController m : nonDriveMotors.values()) {
                m.closeLog();
            }
            imu.closeLog();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}