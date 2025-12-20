package com.epra.epralib.ftclib.sensor;

import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileReader;
import java.util.HashMap;

/// A record that stores settings for a [MultiColorSensor].
///
/// These settings are the darkest value (`black`), the lightest value (`white`) that the color sensors can detect,
/// and any other colors normalized between black and white to compare color sensor readings against.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class ColorSensorSettings {

    private final Vector black;
    private final Vector white;
    private final HashMap<String, Vector> colors = new HashMap<>();

    /// A record that stores settings for a [MultiColorSensor].
    ///
    /// These settings are the darkest value (`black`), the lightest value (`white`) that the color sensors can detect,
    /// and any other colors normalized between black and white to compare color sensor readings against
    /// @param black A normalized RGB array representing the `black` value
    /// @param white A normalized RGB array representing the `white` value
    /// @param colors Names paired with RGB arrays normalized relative to the given `white` and black `values` representing
    /// reference colors to compare the color sensor readings against
    public ColorSensorSettings(double[] black, double[] white, HashMap<String, double[]> colors) {
        this.black = new Vector(black[0], black[1], black[2]);
        this.white = new Vector(white[0], white[1], white[2]);
        for (String s : colors.keySet()) {
            this.colors.put(s, new Vector(colors.get(s)[0], colors.get(s)[1], colors.get(s)[2]));
        }
    }

    /// Returns the RGB value of the darkest value (`black`) that the color sensors can detect as a [Vector].
    /// @return The `black` value
    public Vector black() { return black; }
    /// Returns the RGB value of the lightest value (`white`) that the color sensors can detect as a [Vector].
    /// @return The `white` value
    public Vector white() { return white; }

    /// Returns the [HashMap] that relates a color name to a [Vector] with that color's RGB value normalized
    /// between `black` and `white` values.
    /// @return The hash map of color names and normalized color values
    public HashMap<String, Vector> colorMap() { return colors; }
    /// Returns the array of all color names.
    /// @return The array of all color names
    public String[] colorNames() { return colors.keySet().toArray(new String[0]); }
    /// Returns the array of all color RGB values as [Vector]s normalized between the `black` and `white` values.
    /// @return The array of normalized color values
    public Vector[] colors() { return colors.values().toArray(new Vector[0]); }

    /// Loads a [ColorSensorSettings] record the given `filename`.
    ///
    /// Will return `null` and log an error if the `filename` cannot be found.
    /// @param filename The filename of the `JSON` file with the color sensor settings
    /// @return The color sensor settings from the file
    public static ColorSensorSettings fromFile(String filename) {
        try (FileReader file = new FileReader(AppUtil.getInstance().getSettingsFile(filename))) {
            Gson gson = new Gson();
            return gson.fromJson(file, new TypeToken<ColorSensorSettings>() {}.getType());
        } catch (Exception e) {
            LogController.logError("Could not load encoder displacements from " + filename +
                    ". Error: " + e.getMessage());
        }
        return null;
    }
}
