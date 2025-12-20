package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.*;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.math.statistics.RollingAverage;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.epra.epralib.ftclib.movement.DriveTrain;
import com.epra.epralib.ftclib.movement.MotorController;
import com.epra.epralib.ftclib.storage.logdata.DataLogger;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.*;
import java.util.function.Supplier;

/// Uses [MotorController] encoders as odometry pods to determine the robot's [Pose].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class Odometry implements DataLogger {

    //private final double INCH_PER_TICK = (1.88976 * Math.PI) / 2000.0;

    public enum Orientation { LEFT, RIGHT, PERPENDICULAR }

    public enum LoggingTarget { X, Y }

    private final Map<Orientation, Supplier<Double>> encoder = new HashMap<>();
    private final Map<Orientation, Double> unitPerTick = new HashMap<>();
    private final Map<Orientation, Vector> displacement = new HashMap<>();
    private final Map<Orientation, Double> last = new HashMap<>();
    private final Map<Orientation, Double> delta = new HashMap<>();
    private Angle phi = new Angle();
    private final Pose startPose;

    private final Supplier<Angle> heading;
    protected boolean headingEnabled;

    private Pose pose;
    private Vector deltaPose;

    private long saveTime;

    /// A buffer for velocity (Δ position) values from the robot.
    private final RollingAverage velocityBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);
    /// A buffer for the angle of the velocity (Φ) value from the robot in radians.
    private final RollingAverage phiBuffer = new RollingAverage(25, RollingAverage.Bias.LINEAR);

    private final String logPath;
    private final HashMap<String, Double> logData;
    private final LoggingTarget[] loggingTargets;

    /// Uses [MotorController] encoders as odometry pods to determine the robot's [Pose].
    ///
    /// All measurements should be in consistent units, but the calculations are independent of the units used.
    /// @param leftEncoder A supplier for the position of the left parallel encoder
    /// @param rightEncoder A supplier for the position of the right parallel encoder
    /// @param perpendicularEncoder A supplier for the position of the perpendicular encoder
    /// @param leftUnitPerTick The units per tick for the left encoder
    /// @param rightUnitPerTick The units per tick for the right encoder
    /// @param perpendicularUnitPerTick The units per tick for the perpendicular encoder
    /// @param displacementLeft The displacement from the robot center of the left encoder
    /// @param displacementRight The displacement from the robot center of the right encoder
    /// @param displacementPerpendicular The displacement from the robot center of the perpendicular encoder
    /// @param heading A supplier for the current heading of the robot from an IMU or multiIMU
    /// @param startPose The starting pose of the robot on the field
    public Odometry(Supplier<Double> leftEncoder, Supplier<Double> rightEncoder, Supplier<Double> perpendicularEncoder,
                    double leftUnitPerTick, double rightUnitPerTick, double perpendicularUnitPerTick,
                    Vector displacementLeft, Vector displacementRight, Vector displacementPerpendicular,
                    Supplier<Angle> heading, Pose startPose, LoggingTarget[] loggingTargets) {
        encoder.put(Orientation.LEFT, leftEncoder);
        encoder.put(Orientation.RIGHT, rightEncoder);
        unitPerTick.put(Orientation.LEFT, leftUnitPerTick);
        unitPerTick.put(Orientation.RIGHT, rightUnitPerTick);
        unitPerTick.put(Orientation.PERPENDICULAR, perpendicularUnitPerTick);
        encoder.put(Orientation.PERPENDICULAR, perpendicularEncoder);
        displacement.put(Orientation.LEFT, displacementLeft);
        displacement.put(Orientation.RIGHT, displacementRight);
        displacement.put(Orientation.PERPENDICULAR, displacementPerpendicular);
        this.heading = heading;
        this.headingEnabled = true;
        pose = startPose;
        this.startPose = startPose;

        for (Map.Entry<Orientation, Supplier<Double>> entry : encoder.entrySet()) {
            last.put(entry.getKey(), entry.getValue().get());
            delta.put(entry.getKey(), 0.0);
        }
        saveTime = System.currentTimeMillis();

        this.logPath = "Odometry";
        this.logData = new HashMap<>();
        this.loggingTargets = loggingTargets;
    }

    /// Uses [MotorController] encoders as odometry pods to determine the robot's [Pose].
    ///
    /// All measurements should be in consistent units, but the calculations are independent of the units used.
    ///
    /// An [EncoderSettings] record is used to set encoder displacements and units per tick.
    /// @param leftEncoder A supplier for the position of the left parallel encoder
    /// @param rightEncoder A supplier for the position of the right parallel encoder
    /// @param perpendicularEncoder A supplier for the position of the perpendicular encoder
    /// @param encoderSettings A record containing settings to initialize the odometry
    /// @param heading A supplier for the current heading of the robot from an IMU or multiIMU
    /// @param startPose The starting pose of the robot on the field
    public Odometry(Supplier<Double> leftEncoder, Supplier<Double> rightEncoder, Supplier<Double> perpendicularEncoder,
                    EncoderSettings encoderSettings,
                    Supplier<Angle> heading, Pose startPose, LoggingTarget[] loggingTargets) {
        this(leftEncoder, rightEncoder, perpendicularEncoder,
                encoderSettings.leftUnitsPerTick(), encoderSettings.rightUnitsPerTick(), encoderSettings.perpendicularUnitsPerTick(),
                encoderSettings.left(), encoderSettings.right(), encoderSettings.perpendicular(),
                heading, startPose, loggingTargets);
    }

    /// Uses [MotorController] encoders as odometry pods to determine the robot's [Pose].
    ///
    /// All measurements should be in consistent units, but the calculations are independent of the units used.
    ///
    /// This version of odometry doesn't use a perpendicular encoder, so it is only intended to be used in conjunction
    /// with a non-holonomic [DriveTrain#DriveType].
    /// @param leftEncoder A supplier for the position of the left parallel encoder
    /// @param rightEncoder A supplier for the position of the right parallel encoder
    /// @param leftUnitPerTick The units per tick for the left encoder
    /// @param rightUnitPerTick The units per tick for the right encoder
    /// @param displacementLeft The displacement from the robot center of the left encoder
    /// @param displacementRight The displacement from the robot center of the right encoder
    /// @param heading A supplier for the current heading of the robot from an IMU or multiIMU
    /// @param startPose The starting pose of the robot on the field
    public Odometry(Supplier<Double> leftEncoder, Supplier<Double> rightEncoder,
                    double leftUnitPerTick, double rightUnitPerTick,
                    Vector displacementLeft, Vector displacementRight,
                    Supplier<Angle> heading, Pose startPose, LoggingTarget[] loggingTargets) {
        encoder.put(Orientation.LEFT, leftEncoder);
        encoder.put(Orientation.RIGHT, rightEncoder);
        unitPerTick.put(Orientation.LEFT, leftUnitPerTick);
        unitPerTick.put(Orientation.RIGHT, rightUnitPerTick);
        displacement.put(Orientation.LEFT, displacementLeft);
        displacement.put(Orientation.RIGHT, displacementRight);
        this.heading = heading;
        this.headingEnabled = true;
        pose = startPose;
        this.startPose = startPose;

        for (Map.Entry<Orientation, Supplier<Double>> entry : encoder.entrySet()) {
            last.put(entry.getKey(), entry.getValue().get());
            delta.put(entry.getKey(), 0.0);
        }
        saveTime = System.currentTimeMillis();

        this.logPath = "Odometry";
        this.logData = new HashMap<>();
        this.loggingTargets = loggingTargets;
    }

    /// Uses [MotorController] encoders as odometry pods to determine the robot's [Pose].
    ///
    /// All measurements should be in consistent units, but the calculations are independent of the units used.
    ///
    /// This version of odometry doesn't use a perpendicular encoder, so it is only intended to be used in conjunction
    ///
    /// An [EncoderSettings] record is used to set encoder displacements and units per tick.
    /// @param leftEncoder A supplier for the position of the left parallel encoder
    /// @param rightEncoder A supplier for the position of the right parallel encoder
    /// @param encoderSettings A record containing settings to initialize the odometry
    /// @param heading A supplier for the current heading of the robot from an IMU or multiIMU
    /// @param startPose The starting pose of the robot on the field
    public Odometry(Supplier<Double> leftEncoder, Supplier<Double> rightEncoder,
                    EncoderSettings encoderSettings,
                    Supplier<Angle> heading, Pose startPose, LoggingTarget[] loggingTargets) {
        this(leftEncoder, rightEncoder,
                encoderSettings.leftUnitsPerTick(), encoderSettings.rightUnitsPerTick(),
                encoderSettings.left(), encoderSettings.right(),
                heading, startPose, loggingTargets);
    }

    /// A builder for [Odometry].
    public static class Builder {
        private final Map<Orientation, Supplier<Double>> encoder = new HashMap<>();
        private final Map<Orientation, Double> unitPerTick = new HashMap<>();
        private final Map<Orientation, Vector> displacement = new HashMap<>();
        private Supplier<Angle> heading;
        private Pose startPose;
        private final ArrayList<LoggingTarget> loggingTargets = new ArrayList<>();

        /// A builder for [Odometry].
        ///
        /// Starts with default parameters:
        /// - No [MotorController] encoders.
        /// - No heading [Angle] supplier.
        /// - Start pose at (0,0) at an angle of 0 degrees.
        /// - No logging targets.
        /// **A [#rightEncoder(Supplier, double, Vector)] and [#leftEncoder(Supplier, double, Vector)]
        /// MUST be set or the odometry will not be able to function.**
        /// @see #perpendicularEncoder(Supplier, double, Vector)
        /// @see #heading(Supplier)
        /// @see #startPose(Pose)
        /// @see #loggingTargets(LoggingTarget...)
        /// @see #build()
        public Builder() {
            startPose = new Pose(new Vector(0, 0), new Angle());
            heading = null;
        }
        /// Sets the left parallel encoder for the [Odometry].
        /// @param encoderSupplier A supplier for the position of the left encoder
        /// @param unitPerTick The units per tick for the left encoder
        /// @param displacement The displacement from the robot center of the left encoder
        /// @return This builder
        public Builder leftEncoder(Supplier<Double> encoderSupplier, double unitPerTick, Vector displacement) {
            this.encoder.put(Orientation.LEFT, encoderSupplier);
            this.unitPerTick.put(Orientation.LEFT, unitPerTick);
            this.displacement.put(Orientation.LEFT, displacement);
            return this;
        }
        /// Sets the right parallel encoder for the [Odometry].
        /// @param encoderSupplier A supplier for the position of the right encoder
        /// @param unitPerTick The units per tick for the right encoder
        /// @param displacement The displacement from the robot center of the right encoder
        /// @return This builder
        public Builder rightEncoder(Supplier<Double> encoderSupplier, double unitPerTick, Vector displacement) {
            this.encoder.put(Orientation.RIGHT, encoderSupplier);
            this.unitPerTick.put(Orientation.RIGHT, unitPerTick);
            this.displacement.put(Orientation.RIGHT, displacement);
            return this;
        }
        /// Sets the perpendicular encoder for the [Odometry].
        /// @param encoderSupplier A supplier for the position of the perpendicular encoder
        /// @param unitPerTick The units per tick for the perpendicular encoder
        /// @param displacement The displacement from the robot center of the perpendicular encoder
        /// @return This builder
        public Builder perpendicularEncoder(Supplier<Double> encoderSupplier, double unitPerTick, Vector displacement) {
            this.encoder.put(Orientation.PERPENDICULAR, encoderSupplier);
            this.unitPerTick.put(Orientation.PERPENDICULAR, unitPerTick);
            this.displacement.put(Orientation.PERPENDICULAR, displacement);
            return this;
        }
        /// Uses a [EncoderSettings] record to initialize displacements and units per tick for the encoders.
        ///
        /// This should be used **after** adding the encoders as adding encoders will override these settings.
        /// @param encoderSettings The encoder settings to be used
        /// @return This builder
        public Builder useEncoderSettings(EncoderSettings encoderSettings) {
            displacement.put(Orientation.LEFT, encoderSettings.left());
            displacement.put(Orientation.RIGHT, encoderSettings.right());
            displacement.put(Orientation.PERPENDICULAR, encoderSettings.perpendicular());

            unitPerTick.put(Orientation.LEFT, encoderSettings.leftUnitsPerTick());
            unitPerTick.put(Orientation.RIGHT, encoderSettings.rightUnitsPerTick());
            unitPerTick.put(Orientation.PERPENDICULAR, encoderSettings.perpendicularUnitsPerTick());
            return this;
        }
        /// Loads a [EncoderSettings] record the given `filename` and uses that record to initialize
        /// the displacements and units per tick for the encoders.
        ///
        /// This should be used **after** adding the encoders as adding encoders will override these settings.
        ///
        /// Will return `null` and log an error if the `filename` cannot be found.
        /// @param filename The filename of the `JSON` file with the encoder displacements
        /// @return This builder
        public Builder useEncoderSettingsFile(String filename) {
            EncoderSettings encoderSettings = EncoderSettings.fromFile(filename);
            if (encoderSettings == null) { return null; }
            return useEncoderSettings(encoderSettings);
        }

        /// Sets a heading [Angle] supplier for the [Odometry].
        ///
        /// Should in most cases come from an [IMU] or [MultiIMU].
        /// @param headingSupplier A supplier for the current heading of the robot
        /// @return This builder
        public Builder heading(Supplier<Angle> headingSupplier) {
            this.heading = headingSupplier;
            return this;
        }
        /// Sets a start [Pose] for the [Odometry].
        /// @param startPose The starting pose of the robot on the field
        /// @return This builder
        public Builder startPose(Pose startPose) {
            this.startPose = startPose;
            return this;
        }
        /// Adds any number of [LoggingTarget]s for the [Odometry] to log.
        /// @param loggingTargets Any number of logging targets
        /// @return This builder
        public Builder loggingTargets(LoggingTarget... loggingTargets) {
            this.loggingTargets.addAll(Arrays.asList(loggingTargets));
            return this;
        }

        /// Builds an [Odometry] from this builder.
        ///
        /// Will return `null` if either the [#leftEncoder(Supplier, double, Vector) or
        /// the [#rightEncoder(Supplier, double, Vector)] were not set.
        /// @return The odometry built from this builder
        public Odometry build() {
            if (!encoder.containsKey(Orientation.LEFT) || !encoder.containsKey(Orientation.RIGHT)) return null;
            Odometry o;
            if (encoder.containsKey(Orientation.PERPENDICULAR)) {
                o = new Odometry(
                        encoder.get(Orientation.LEFT), encoder.get(Orientation.RIGHT), encoder.get(Orientation.PERPENDICULAR),
                        unitPerTick.get(Orientation.LEFT), unitPerTick.get(Orientation.RIGHT), unitPerTick.get(Orientation.PERPENDICULAR),
                        displacement.get(Orientation.LEFT), displacement.get(Orientation.RIGHT), displacement.get(Orientation.PERPENDICULAR),
                        heading, startPose, loggingTargets.toArray(new LoggingTarget[0])
                );
            } else {
                o = new Odometry(
                        encoder.get(Orientation.LEFT), encoder.get(Orientation.RIGHT),
                        unitPerTick.get(Orientation.LEFT), unitPerTick.get(Orientation.RIGHT),
                        displacement.get(Orientation.LEFT), displacement.get(Orientation.RIGHT),
                        heading, startPose, loggingTargets.toArray(new LoggingTarget[0])
                );
            }
            o.headingEnabled = heading == null;
            return o;
        }
    }

    /// Updates delta pos and save pos for all encoders.
    private void updateDeltaPos() {
        for (Map.Entry<Orientation, Supplier<Double>> entry : encoder.entrySet()) {
            double current = entry.getValue().get();
            delta.replace(entry.getKey(), current - last.get(entry.getKey()));
            last.replace(entry.getKey(), current);
        }
    }

    /// Updates the Φ (Δθ) value based on measurements from the encoders.
    /// @return The new Φ value
    private Angle phiEncoder() {
        double l = Math.abs(Geometry.subtract(displacement.get(Orientation.LEFT), displacement.get(Orientation.RIGHT)).x());
        phi = Angle.degree((((delta.get(Orientation.RIGHT) * unitPerTick.get(Orientation.RIGHT) -
                delta.get(Orientation.LEFT) * unitPerTick.get(Orientation.LEFT)))
                / l) * (180.0 / Math.PI) * -1.0);
        //compensates for previous error
        Angle diff = Geometry.subtract(Geometry.add(Angle.degree((((last.get(Orientation.RIGHT) * unitPerTick.get(Orientation.RIGHT)
                - last.get(Orientation.LEFT) * unitPerTick.get(Orientation.LEFT))) / l) * (180.0 / Math.PI) * -1.0), startPose.angle), this.pose.angle);
        phi = Geometry.add(diff, phi);
        return phi;
    }
    /// Updates the Φ (Δθ) value based on the heading supplier.
    /// @return The new Φ value
    private Angle phiHeading() {
        phi = Geometry.subtract(heading.get(), pose.angle);
        return phi;
    }

    /// Calculates the center displacement of the parallel encoders
    /// by taking an average of the individual displacements.
    /// @return The calculated center displacement
    private double centerDisplacement() { return (delta.get(Orientation.LEFT) + delta.get(Orientation.RIGHT)) / 2.0; }
    /// Calculates the center units per tick of the parallel encoders
    /// by taking an average of the individual units per tick.
    /// @return The calculated center units per tick
    private double centerUnitsPerTick() { return (unitPerTick.get(Orientation.LEFT) + unitPerTick.get(Orientation.RIGHT)) / 2.0; }
    /// Calculates the displacement of the perpendicular encoder if it is present.
    ///
    /// If the perpendicular encoder is not set, will always return 0.
    /// @return The perpendicular displacement.
    private double perpendicularDisplacement() {
        if (encoder.containsKey(Orientation.PERPENDICULAR))
            return delta.get(Orientation.PERPENDICULAR) - (displacement.get(Orientation.PERPENDICULAR).y() * phi.radian());
        return 0.0;
    }

    /// Calculates the current [Pose] of the robot based on encoder measurements and the last calculated pose.
    /// @return The calculated pose
    public Pose estimatePose() {
        Pose lastPose = pose;
        updateDeltaPos();
        if (headingEnabled) {
            phiHeading();
        } else {
            phiEncoder();
        }
        Vector p0 = new Vector(
                centerDisplacement(),
                perpendicularDisplacement()
        );
        Vector p1 = new Vector(
                (p0.x() * Geometry.cos(pose.angle)) * centerUnitsPerTick() -
                        (p0.y() * Geometry.sin(pose.angle) * unitPerTick.get(Orientation.PERPENDICULAR)),
                (p0.x() * Geometry.sin(pose.angle)) * centerUnitsPerTick() +
                        (p0.y() * Geometry.cos(pose.angle) * unitPerTick.get(Orientation.PERPENDICULAR))
        );
        double phiRadians = (phi.radian() != 0.0) ? phi.radian() : 1.0f;
        Vector p2 = new Vector(
                (((p1.x()* (1.0 - Geometry.cos(phi)) / phiRadians)) +
                        (p1.y()* (Geometry.sin(phi)) / phiRadians)),
                (((p1.x()* (Geometry.sin(phi)) / phiRadians) +
                        (p1.y()* (Geometry.cos(phi) - 1.0) / phiRadians))) * -1.0
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

        return pose;
    }

    /// Returns the most recently calculated [Pose].
    ///
    /// [#estimatePose()] must be called frequently for this value to be accurate.
    /// @return The most recently calculated pose
    public Pose getPose() { return pose; }

    /// Returns the difference between the two most recently calculated [Pose]s as a 2D [Vector].
    ///
    /// [#estimatePose()] must be called frequently for this value to be meaningful.
    /// @return The most recent change in position
    public Vector getDeltaPose() { return deltaPose; }

    /// Returns the recent average velocity of the robot as a 2D [Vector] in units per second.
    ///
    /// [#estimatePose()] must be called frequently for this value to be accurate.
    /// @return The recent average velocity of the robot
    public Vector getVelocity() { return new Vector(velocityBuffer.getAverage(), Angle.radian(phiBuffer.getAverage())); }
    /// Returns the recent average acceleration of the robot as a 2D [Vector] in units per second².
    ///
    /// [#estimatePose()] must be called frequently for this value to be accurate.
    /// @return The recent average acceleration of the robot
    public Vector getAcceleration() {
        RollingAverage vd = velocityBuffer.getDerivative();
        RollingAverage ad = phiBuffer.getDerivative();
        return new Vector(vd.getAverage(), Angle.radian(ad.getAverage()));
    }

    /// Draws the robot, its velocity, acceleration, and odometer velocity onto the field map.
    /// @param robotShape A quadrilateral representing the 2D shape of the robot relative to the center of the robot
    /// @param drawOdometers Whether to draw the odometers and their velocities
    /// @return A packet modified to contain a drawing of the robot pose onto the field map
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

    /// {@inheritDoc}
    /// @return The relative file path for the logs from this logger
    @Override
    public String getLogPath() { return "Odometry.json"; }
    /// {@inheritDoc}
    /// @return `True`
    @Override
    public boolean updateLog() {
        Pose pose = estimatePose();
        for (LoggingTarget target : loggingTargets) {
            logData.put(target.toString().toLowerCase(), switch (target) {
                case X -> pose.pos.x();
                case Y -> pose.pos.y();
                default -> 0.0;
            });
        }
        return true;
    }
    /// {@inheritDoc}
    /// @return A hash map with a data snapshot of this logger
    @Override
    public HashMap<String, Double> logData() { return logData; }
    /// {@inheritDoc}
    /// @return The unix time in milliseconds when the ping was received
    @Override
    public long ping() { return System.currentTimeMillis(); }
}