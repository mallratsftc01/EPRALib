package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.*;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.math.statistics.RollingAverage;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.epra.epralib.ftclib.storage.initialization.EncoderDisplacements;
import com.google.gson.Gson;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.function.Supplier;

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

    private final Map<Orientation, Supplier<Double>> encoder = new HashMap<>();
    private final Map<Orientation, Vector> displacement = new HashMap<>();
    private final Map<Orientation, Double> last = new HashMap<>();
    private final Map<Orientation, Double> delta = new HashMap<>();
    private Angle phi = new Angle();
    private final Pose startPose;

    private final Supplier<Angle> heading;

    private Pose pose;
    private Vector deltaPose;

    private long saveTime;

    /**A buffer for velocity (delta position) values from the robot.*/
    private final RollingAverage velocityBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);
    /**A buffer for the angle of the velocity (phi) value from the robot in radians.*/
    private final RollingAverage phiBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);

    private final File logJson;
    private final FileWriter logWriter;
    private final Gson gson;

    /**Uses odometer encoders to determine robot pose.
     * @param leftEncoder A supplier for the position of the left parallel encoder.
     * @param rightEncoder A supplier for the position of the right parallel encoder.
     * @param perpendicularEncoder A supplier for the position of the perpendicular encoder.
     * @param displacementLeft The displacement from the robot center of the left encoder in inches.
     * @param displacementRight The displacement from the robot center of the right encoder in inches.
     * @param displacementPerpendicular The displacement from the robot center of the perpendicular encoder in inches.
     * @param heading A supplier for the current heading of the robot from the imu.
     * @param startPose The starting pose of the robot on the field.
     * */
    public Odometry(Supplier<Double> leftEncoder, Supplier<Double> rightEncoder, Supplier<Double> perpendicularEncoder, Vector displacementLeft, Vector displacementRight, Vector displacementPerpendicular, Supplier<Angle> heading, Pose startPose) throws IOException {
        encoder.put(Orientation.LEFT, leftEncoder);
        encoder.put(Orientation.RIGHT, rightEncoder);
        encoder.put(Orientation.PERPENDICULAR, perpendicularEncoder);
        displacement.put(Orientation.LEFT, displacementLeft);
        displacement.put(Orientation.RIGHT, displacementRight);
        displacement.put(Orientation.PERPENDICULAR, displacementPerpendicular);
        this.heading = heading;
        pose = startPose;
        this.startPose = startPose;

        for (Map.Entry<Orientation, Supplier<Double>> entry : encoder.entrySet()) {
            last.put(entry.getKey(), entry.getValue().get());
            delta.put(entry.getKey(), 0.0);
        }
        saveTime = System.currentTimeMillis();

        gson = new Gson();

        SimpleDateFormat ft = new SimpleDateFormat("ddMMyyyy:HH:mm", Locale.US);
        logJson = AppUtil.getInstance().getSettingsFile("logs/Pose_log_" + ft.format(new Date()) + ".json");
        logWriter = new FileWriter(logJson, true);
        logWriter.write("[");
    }

    /**Uses odometer encoders to determine robot pose.
     * @param leftEncoder A supplier for the position of the left parallel encoder.
     * @param rightEncoder A supplier for the position of the right parallel encoder.
     * @param perpendicularEncoder A supplier for the position of the perpendicular encoder.
     * @param encoderDisplacements A record containing the displacements of the encoders from the center of the robot in inches.
     * @param heading A supplier for the current heading of the robot from the imu.
     * @param startPose The starting pose of the robot on the field.
     * */
    public Odometry(Supplier<Double> leftEncoder, Supplier<Double> rightEncoder, Supplier<Double> perpendicularEncoder, EncoderDisplacements encoderDisplacements, Supplier<Angle> heading, Pose startPose) throws IOException {
        this(leftEncoder, rightEncoder, perpendicularEncoder, encoderDisplacements.left(), encoderDisplacements.right(), encoderDisplacements.perpendicular(), heading, startPose);
    }

    /**Updates the delta and pos save of the encoders.*/
    public void updateDeltaPos() {
        for (Map.Entry<Orientation, Supplier<Double>> entry : encoder.entrySet()) {
            double current = entry.getValue().get();
            delta.replace(entry.getKey(), current - last.get(entry.getKey()));
            last.replace(entry.getKey(), current);
        }
    }
    /**@return The pos value associated with the corresponding encoder.*/
    public double getPos(Orientation key) { return encoder.get(key).get(); }

    /**@return The change in pos value associated with the corresponding encoder.*/
    public double getDelta(Orientation key) { return delta.get(key); }

    /**Updates the phi (delta theta) value using the encoders.
     * @return The new phi value. */
    public Angle phiEncoder() {
        double l = Math.abs(Geometry.subtract(displacement.get(Orientation.LEFT), displacement.get(Orientation.RIGHT)).x());
        phi = Angle.degree((((delta.get(Orientation.RIGHT) - delta.get(Orientation.LEFT)) * INCH_PER_TICK) / l) * (180.0 / Math.PI) * -1.0);
        //compensates for previous error
        Angle diff = Geometry.subtract(Geometry.add(Angle.degree((((last.get(Orientation.RIGHT) - last.get(Orientation.LEFT)) * INCH_PER_TICK) / l) * (180.0 / Math.PI) * -1.0), startPose.angle), this.pose.angle);
        phi = Geometry.add(diff, phi);
        return phi;
    }
    /**Updates the phi (delta theta) value using the imu.
     * @return The new phi value. */
    public Angle phiIMU() {
        phi = Geometry.subtract(heading.get(), pose.angle);
        return phi;
    }
    /**@return The rotational change of the robot.*/
    public Angle getPhi() { return phi; }

    /**Finds the center displacement of the parallel encoders.*/
    public double centerDisplacement() { return (delta.get(Orientation.LEFT) + delta.get(Orientation.RIGHT)) / 2.0; }
    /**Finds the displacement of the perpendicular encoder.*/
    public double perpendicularDisplacement() { return delta.get(Orientation.PERPENDICULAR) - (displacement.get(Orientation.PERPENDICULAR).y()* phi.radian()); }

    /**Estimates the new pose value. Store the new pose in a JSON log file.
     * @return The new pose value.*/
    public Pose estimatePose() throws IOException {
        Pose lastPose = pose;
        updateDeltaPos();
        phiEncoder();
        Vector p0 = new Vector(
                centerDisplacement(),
                perpendicularDisplacement()
        );
        Vector p1 = new Vector(
                (p0.x()* Geometry.cos(pose.angle)) - (p0.y() * Geometry.sin(pose.angle)),
                (p0.x()* Geometry.sin(pose.angle)) + (p0.y() * Geometry.cos(pose.angle))
        );
        double phiRadians = (phi.radian() != 0.0) ? phi.radian() : 1.0f;
        Vector p2 = new Vector(
                (((p1.x()* (1.0 - Geometry.cos(phi)) / phiRadians)) + (p1.y()* (Geometry.sin(phi)) / phiRadians)) * INCH_PER_TICK,
                (((p1.x()* (Geometry.sin(phi)) / phiRadians) + (p1.y()* (Geometry.cos(phi) - 1.0) / phiRadians))) * INCH_PER_TICK * -1.0
        );
        pose = new Pose(
                Geometry.add(pose.pos, p2),
                Geometry.add(pose.angle, phi)
        );
        double time = (System.currentTimeMillis() - saveTime) / 1000.0;
        velocityBuffer.addValue(Geometry.pythagorean(p2.x(), p2.y()) / time);
        phiBuffer.addValue(Geometry.add(phi, p2.theta()).radian() / time);
        saveTime = System.currentTimeMillis();

        if (lastPose != null) {
            deltaPose = new Vector(lastPose.pos, pose.pos);
        }

        logWriter.write("\n" + gson.toJson(pose.toPoseData()) + ",");

        return pose;
    }

    /**@return The most recent pose value.*/
    public Pose getPose() { return pose; }

    /**@return the difference between the last two estimated poses as a vector.*/
    public Vector getDeltaPose() { return deltaPose; }

    /**Returns the velocity of the robot as a vector (inch/second).*/
    public Vector getVelocity() { return new Vector(velocityBuffer.getAverage(), Angle.radian(phiBuffer.getAverage())); }
    /**Returns the acceleration of the robot as a vector (inch/second²).*/
    public Vector getAcceleration() {
        RollingAverage vd = velocityBuffer.getDerivative();
        RollingAverage ad = phiBuffer.getDerivative();
        return new Vector(vd.getAverage(), Angle.radian(ad.getAverage()));
    }

    /**Draws the robot, its velocity, acceleration, and odometer velocity onto the field map.
     * @param robotShape A quadrilateral representing the 2D shape of the robot relative to the center of the robot.
     * @param drawOdometers Whether to draw the odometers and their velocities.
     * @return A packet modified to contain a drawing of the robot pose onto the field map.*/
    public TelemetryPacket drawPose(Quadrilateral robotShape, boolean drawOdometers) {
        TelemetryPacket p = new TelemetryPacket();
        Quadrilateral shape = new Quadrilateral(
                Geometry.multiply(Matrix.yaw(pose.angle), robotShape.getA()),
                Geometry.multiply(Matrix.yaw(pose.angle), robotShape.getB()),
                Geometry.multiply(Matrix.yaw(pose.angle), robotShape.getC()),
                Geometry.multiply(Matrix.yaw(pose.angle), robotShape.getD())
        );
        double[] shapeX = {shape.getA().x(), shape.getB().x(), shape.getC().x(), shape.getD().x()};
        double[] shapeY = {shape.getA().y(), shape.getB().y(), shape.getC().y(), shape.getD().y()};
        Vector velocity = Geometry.add(pose.pos, getVelocity());
        Vector acceleration = Geometry.add(velocity, getAcceleration());
        p.fieldOverlay()
                //draws center Vector
                .setFill("blue")
                .fillCircle(pose.pos.x(), pose.pos.y(), 1)
                //draws robot outline
                .fillPolygon(Arrays.copyOfRange(shapeX, 0, 1), Arrays.copyOfRange(shapeY, 0, 1))
                .fillPolygon(Arrays.copyOfRange(shapeX, 1, 2), Arrays.copyOfRange(shapeY, 1, 2))
                .fillPolygon(Arrays.copyOfRange(shapeX, 2, 3), Arrays.copyOfRange(shapeY, 2, 3))
                .fillPolygon(new double[] {shapeX[3], shapeX[0]}, new double[] {shapeY[3], shapeY[0]})
                //draw velocity
                .setFill("green")
                .fillPolygon(new double[] {pose.pos.x(), velocity.x()}, new double[] {pose.pos.y(), velocity.y()})
                //draw acceleration
                .setFill("red")
                .fillPolygon(new double[] {acceleration.x(), velocity.x()}, new double[] {acceleration.y(), velocity.y()});
        if (drawOdometers) {
            //draw odometer velocities
            for (Map.Entry<Orientation, Supplier<Double>> entry : encoder.entrySet()) {
                Vector start = Geometry.add(pose.pos, displacement.get(entry.getKey()));
                Vector velo = new Vector(delta.get(entry.getKey()), Geometry.add(pose.angle, (entry.getKey() == Orientation.PERPENDICULAR) ? Angle.degree(90.0) : new Angle()));
                Vector end = Geometry.add(start, velo);
                p.fieldOverlay()
                        .setFill("green")
                        .fillPolygon(new double[] {start.x(), end.x()}, new double[] { start.y(), end.y()});
            }
        }
        return p;
    }

    /**Closes the JSON file that this Odometry is writing to.*/
    public void closeLog() throws IOException {
        logWriter.write("]");
        logWriter.close();
    }

}
