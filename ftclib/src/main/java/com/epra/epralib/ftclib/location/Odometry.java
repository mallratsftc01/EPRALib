package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Point;
import com.epra.epralib.ftclib.math.geometry.Quadrilateral;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.math.statistics.RollingAverage;
import com.epra.epralib.ftclib.movement.MotorController;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.google.gson.Gson;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

/**Uses odometer encoders to determine robot pose.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Odometry {

    private final double INCH_PER_TICK = (1.88976 * Math.PI) / 2000.0;
    private final double ANGLE_ERROR_OFFSET = (178.4196476 / 100.7794981);

    private final Vector[] TEST_VECTORS = {
            new Vector(1, 0),
            new Vector(1, 1),
            new Vector(0, 1),
            new Vector(-1, 1),
            new Vector(-1, 0),
            new Vector(-1, -1),
            new Vector(0, -1),
            new Vector(1, -1)
    };

    public enum Orientation { LEFT, RIGHT, PERPENDICULAR }

    private Map<Orientation, MotorController> encoder = new HashMap<>();
    private Map<Orientation, Integer> start = new HashMap<>();
    private Map<Orientation, Point> displacement = new HashMap<>();
    private Map<Orientation, Integer> pos = new HashMap<>();
    private Map<Orientation, Integer> delta = new HashMap<>();
    private Angle phi = new Angle(0.0);
    private Pose startPose;

    private IMUExpanded imu;

    private Pose pose;
    private Vector deltaPose;

    private long saveTime;

    /**A buffer of the velocity (delta position) of the robot.*/
    private RollingAverage velocityBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);
    /**A buffer of the angle of the velocity (phi) of the robot in radians.*/
    private RollingAverage phiBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);

    private File logJson;
    private FileWriter logWriter;
    private Gson gson;

    /**Uses odometer encoders to determine robot pose.
     * @param leftEncoder The left parallel encoder.
     * @param rightEncoder The right parallel encoder.
     * @param perpendicularEncoder The perpendicular encoder.
     * @param displacementLeft The displacement from the robot center of the left encoder in inches.
     * @param displacementRight The displacement from the robot center of the right encoder in inches.
     * @param displacementPerpendicular The displacement from the robot center of the perpendicular encoder in inches.
     * @param imu The imu.
     * @param startPose The starting pose of the robot on the field.
     * */
    public Odometry(MotorController leftEncoder, MotorController rightEncoder, MotorController perpendicularEncoder, Point displacementLeft, Point displacementRight, Point displacementPerpendicular, IMUExpanded imu, Pose startPose) throws IOException {
        encoder.put(Orientation.LEFT, leftEncoder);
        encoder.put(Orientation.RIGHT, rightEncoder);
        encoder.put(Orientation.PERPENDICULAR, perpendicularEncoder);
        start.put(Orientation.LEFT, leftEncoder.getCurrentPosition());
        start.put(Orientation.RIGHT, rightEncoder.getCurrentPosition());
        start.put(Orientation.PERPENDICULAR, perpendicularEncoder.getCurrentPosition());
        displacement.put(Orientation.LEFT, displacementLeft);
        displacement.put(Orientation.RIGHT, displacementRight);
        displacement.put(Orientation.PERPENDICULAR, displacementPerpendicular);
        this.imu = imu;
        pose = startPose;
        this.startPose = startPose;

        for (Map.Entry<Orientation, MotorController> entry : encoder.entrySet()) {
            pos.put(entry.getKey(), entry.getValue().getCurrentPosition() - start.get(entry.getKey()));
        }
        for (Map.Entry<Orientation, MotorController> entry : encoder.entrySet()) {
            delta.put(entry.getKey(), (entry.getValue().getCurrentPosition() - start.get(entry.getKey())) - pos.get(entry.getKey()));
        }
        saveTime = System.currentTimeMillis();

        gson = new Gson();

        SimpleDateFormat ft = new SimpleDateFormat("ddMMyyyy:HH:mm");
        logJson = AppUtil.getInstance().getSettingsFile("logs/Pose_log_" + ft.format(new Date()) + ".json");
        logWriter = new FileWriter(logJson);
        logWriter.write("[");
    }

    /**Updates the delta and pos save of the encoders.*/
    public void updateDeltaPos() {
        for (Map.Entry<Orientation, MotorController> entry : encoder.entrySet()) {
            int current = entry.getValue().getCurrentPosition();
            delta.replace(entry.getKey(), current - start.get(entry.getKey()) - pos.get(entry.getKey()));
            pos.replace(entry.getKey(), current - start.get(entry.getKey()));
        }
    }
    /**@return The pos value assoiated with the corresponding encoder.*/
    public int getPos(Orientation key) { return pos.get(key); }

    /**@return The change in pos value assoiated with the corresponding encoder.*/
    public int getDelta(Orientation key) { return delta.get(key); }

    /**Updates the phi (delta theta) value using the encoders.
     * @return The new phi value. */
    public Angle phiEncoder() {
        double l = Math.abs(Geometry.subtract(displacement.get(Orientation.LEFT), displacement.get(Orientation.RIGHT)).x);
        phi = new Angle((((delta.get(Orientation.RIGHT) - delta.get(Orientation.LEFT)) * INCH_PER_TICK) / l) * (180.0 / Math.PI) * -1.0);
        //compensates for previous error
        Angle diff = Geometry.subtract(Geometry.add(new Angle((((pos.get(Orientation.RIGHT) - pos.get(Orientation.LEFT)) * INCH_PER_TICK) / l) * (180.0 / Math.PI) * -1.0), startPose.angle), this.pose.angle);
        phi = Geometry.add(diff, phi);
        return phi;
    }
    /**Updates the phi (delta theta) value using the imu.
     * @return The new phi value. */
    public Angle phiIMU() {
        phi = Geometry.subtract(imu.getYaw(), pose.angle);
        return phi;
    }
    /**@return The change in angle of the robot.*/
    public Angle getPhi() { return phi; }

    /**Finds the center displacement of the parallel encoders.*/
    public double centerDisplacement() { return (delta.get(Orientation.LEFT) + delta.get(Orientation.RIGHT)) / 2.0; }
    /**Finds the displacement of the perpendicular encoder.*/
    public double perpendicularDisplacement() { return delta.get(Orientation.PERPENDICULAR) - (displacement.get(Orientation.PERPENDICULAR).y * phi.getRadian()); }

    /**Estimates the new pose value. Store the new pose in a json log file.
     * @return The new pose value.*/
    public Pose estimatePose() throws IOException {
        Pose lastPose = pose;
        updateDeltaPos();
        phiEncoder();
        Point p0 = new Point(
                centerDisplacement(),
                perpendicularDisplacement()
        );
        Point p1 = new Point(
                (p0.x * Geometry.cos(pose.angle)) - (p0.y* Geometry.sin(pose.angle)),
                (p0.x * Geometry.sin(pose.angle)) + (p0.y * Geometry.cos(pose.angle))
        );
        double phiRadians = (phi.getRadian() != 0.0) ? phi.getRadian() : 1.0f;
        Point p2 = new Point(
                (((p1.x * (1.0 - Geometry.cos(phi)) / phiRadians)) + (p1.y * (Geometry.sin(phi)) / phiRadians)) * INCH_PER_TICK,
                (((p1.x * (Geometry.sin(phi)) / phiRadians) + (p1.y * (Geometry.cos(phi) - 1.0) / phiRadians))) * INCH_PER_TICK * -1.0
        );
        pose = new Pose(
                Geometry.add(pose.point, p2),
                Geometry.add(pose.angle, phi)
        );
        double time = (System.currentTimeMillis() - saveTime) / 1000.0;
        velocityBuffer.addValue(Geometry.pythagorean(p2.x, p2.y) / time);
        phiBuffer.addValue(Geometry.add(phi, new Vector(p2)).getRadian() / time);
        saveTime = System.currentTimeMillis();

        if (lastPose != null) {
            deltaPose = new Vector(lastPose.point, pose.point);
        }

        logWriter.write("\n" + gson.toJson(pose.toPoseData()) + ",");

        return pose;
    }

    /**@return The most recent pose value.*/
    public Pose getPose() { return pose; }

    /**@return the difference between the last two estimated poses as a vector.*/
    public Vector getDeltaPose() { return deltaPose; }

    /**Returns the velocity of the robot as a vector (inch/second).*/
    public Vector getVelocity() { return new Vector(velocityBuffer.getAverage(), new Angle((float) phiBuffer.getAverage())); }
    /**Returns the acceleration of the robot as a vector (inch/second²).*/
    public Vector getAcceleration() {
        RollingAverage vd = velocityBuffer.getDerivative();
        RollingAverage ad = phiBuffer.getDerivative();
        return new Vector(vd.getAverage(), new Angle((float) ad.getAverage()));
    }

    /**Draws the robot, its velocity, acceleration, and odometer velocity onto the field map.
     * @param robotShape A quadrilateral representing the 2D shape of the robot relative to the center of the robot.
     * @param drawOdometers Whether or not to draw the odometers and their velocities.
     * @return A packet modified to contain a drawing of the robot pose onto the field map.*/
    public TelemetryPacket drawPose(Quadrilateral robotShape, boolean drawOdometers) {
        TelemetryPacket p = new TelemetryPacket();
        Quadrilateral shape = new Quadrilateral(
                Geometry.rotate(robotShape.getA(), pose.angle),
                Geometry.rotate(robotShape.getB(), pose.angle),
                Geometry.rotate(robotShape.getC(), pose.angle),
                Geometry.rotate(robotShape.getD(), pose.angle)
        );
        double[] shapeX = {shape.getA().x, shape.getB().x, shape.getC().x, shape.getD().x};
        double[] shapeY = {shape.getA().y, shape.getB().y, shape.getC().y, shape.getD().y};
        Point velocity = Geometry.add(pose.point, getVelocity().toPoint());
        Point acceleration = Geometry.add(velocity, getAcceleration().toPoint());
        p.fieldOverlay()
                //draws center point
                .setFill("blue")
                .fillCircle(pose.point.x, pose.point.y, 1)
                //draws robot outline
                .fillPolygon(Arrays.copyOfRange(shapeX, 0, 1), Arrays.copyOfRange(shapeY, 0, 1))
                .fillPolygon(Arrays.copyOfRange(shapeX, 1, 2), Arrays.copyOfRange(shapeY, 1, 2))
                .fillPolygon(Arrays.copyOfRange(shapeX, 2, 3), Arrays.copyOfRange(shapeY, 2, 3))
                .fillPolygon(new double[] {shapeX[3], shapeX[0]}, new double[] {shapeY[3], shapeY[0]})
                //draw velocity
                .setFill("green")
                .fillPolygon(new double[] {pose.point.x, velocity.x}, new double[] {pose.point.y, velocity.y})
                //draw acceleration
                .setFill("red")
                .fillPolygon(new double[] {acceleration.x, velocity.x}, new double[] {acceleration.y, velocity.y});
        if (drawOdometers) {
            //draw odometer velocities
            for (Map.Entry<Orientation, MotorController> entry : encoder.entrySet()) {
                Point start = Geometry.add(pose.point, displacement.get(entry.getKey()));
                Vector velo = new Vector(delta.get(entry.getKey()), Geometry.add(pose.angle, (entry.getKey() == Orientation.PERPENDICULAR) ? new Angle(90.0) : new Angle()));
                Point end = Geometry.add(start, velo.toPoint());
                p.fieldOverlay()
                        .setFill("green")
                        .fillPolygon(new double[] {start.x, end.x}, new double[] { start.y, end.y});
            }
        }
        return p;
    }

    /**Closes the json file that this Odometry is writing to.*/
    public void closeLog() throws IOException {
        logWriter.write("]");
        logWriter.close();
    }

}
