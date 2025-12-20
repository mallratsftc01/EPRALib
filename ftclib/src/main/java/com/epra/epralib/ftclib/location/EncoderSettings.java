package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileReader;

/// A record that stores the displacements from the center of the robot and units per tick of encoders for [Odometry].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public record EncoderSettings(double leftX, double leftY, double leftUnitsPerTick,
                              double rightX, double rightY, double rightUnitsPerTick,
                              double perpendicularX, double perpendicularY, double perpendicularUnitsPerTick) {
    /// Returns the displacement of the left encoder as a [Vector].
    /// @return The displacement of the left encoder
    public Vector left() { return new Vector(leftX, leftY); }
    /// Returns the displacement of the right encoder as a [Vector].
    /// @return The displacement of the right encoder
    public Vector right() { return new Vector(rightX, rightY); }
    /// Returns the displacement of the perpendicular encoder as a [Vector].
    /// @return The displacement of the perpendicular encoder
    public Vector perpendicular() { return new Vector(perpendicularX, perpendicularY); }

    /// Loads a [EncoderSettings] record the given `filename`.
    ///
    /// Will return `null` and log an error if the `filename` cannot be found.
    /// @param filename The filename of the `JSON` file with the encoder displacements
    /// @return The encoder displacements from the file
    public static EncoderSettings fromFile(String filename) {
        try (FileReader file = new FileReader(AppUtil.getInstance().getSettingsFile(filename))) {
            Gson gson = new Gson();
            return gson.fromJson(file, new TypeToken<EncoderSettings>() {}.getType());
        } catch (Exception e) {
            LogController.logError("Could not load encoder displacements from " + filename +
                    ". Error: " + e.getMessage());
        }
        return null;
    }
}
