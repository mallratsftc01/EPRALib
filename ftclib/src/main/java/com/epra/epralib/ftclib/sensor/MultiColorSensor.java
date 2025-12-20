package com.epra.epralib.ftclib.sensor;

import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/// Increases the functionality of the [NormalizedColorSensor] and can combine data from multiple color sensors.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class MultiColorSensor {

    private final ArrayList<NormalizedColorSensor> colorSensors = new ArrayList<>();
    private Vector BLACK;
    private Vector WHITE;
    private final HashMap<String, Vector> colorMap = new HashMap<>();

    /// Increases the functionality of the [NormalizedColorSensor] and can combine data from multiple color sensors.
    /// @param blackRGB A normalized RGB vector representing the darkest color that the color sensors can detect
    /// @param whiteRGB A normalized RGB vector representing the brightest color that the color sensors can detect
    /// @param colorNames The names of reference colors to compare color sensor readings against
    /// @param colorsRGB RGB vectors normalized relative to the given white and black values representing
    /// reference colors to compare the color sensor readings against
    /// @param colorSensor A color sensor
    /// @param colorSensors Any number of additional color sensors
    public MultiColorSensor(Vector blackRGB, Vector whiteRGB, String[] colorNames, Vector[] colorsRGB, NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this.colorSensors.add(colorSensor);
        this.colorSensors.addAll(Arrays.asList(colorSensors));
        BLACK = blackRGB;
        WHITE = whiteRGB;
        colorMap.put("Black", new Vector(0, 0, 0));
        colorMap.put("White", new Vector(1, 1, 1));
        for (int i = 0; i < colorNames.length; i++) {
            colorMap.put(colorNames[i], colorsRGB[i]);
        }
    }
    /// Increases the functionality of the [NormalizedColorSensor] and can combine data from multiple color sensors.
    /// @param blackRGB A normalized RGB vector representing the darkest color that the color sensors can detect
    /// @param whiteRGB A normalized RGB vector representing the brightest color that the color sensors can detect
    /// @param colorSensor A color sensor
    /// @param colorSensors Any number of additional color sensors
    public MultiColorSensor(Vector blackRGB, Vector whiteRGB, NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(blackRGB, whiteRGB, new String[0], new Vector[0], colorSensor, colorSensors);
    }
    /// Increases the functionality of the [NormalizedColorSensor] and can combine data from multiple color sensors.
    /// @param colorSensor A color sensor
    /// @param colorSensors Any number of additional color sensors
    public MultiColorSensor(NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(new Vector(0, 0, 0), new Vector(1, 1, 1), colorSensor, colorSensors);
    }
    /// Increases the functionality of the [NormalizedColorSensor] and can combine data from multiple color sensors.
    /// @param colorSensorSettings An object containing settings for this set of color sensors
    /// @param colorSensor A color sensor
    /// @param colorSensors Any number of additional color sensors
    public MultiColorSensor(ColorSensorSettings colorSensorSettings, NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(colorSensorSettings.black(), colorSensorSettings.white(),
                colorSensorSettings.colorNames(), colorSensorSettings.colors(), colorSensor, colorSensors);
    }

    /// Increases the functionality of the [NormalizedColorSensor] and can combine data from multiple color sensors.
    /// @param colorSensorSettingsFilename The filename of a `JSON` file with settings for this set of color sensors
    /// @param colorSensor A color sensor
    /// @param colorSensors Any number of additional color sensors
    public MultiColorSensor(String colorSensorSettingsFilename, NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(ColorSensorSettings.fromFile(colorSensorSettingsFilename), colorSensor, colorSensors);
    }

    /// Averages the RGB readings from all [NormalizedColorSensor]s and returns the average as a [Vector]
    /// @return The average readings of all color sensors
    public Vector rawAvgRGB() {
        double r = 0, g = 0, b = 0;
        for (NormalizedColorSensor colorSensor : colorSensors) {
            NormalizedRGBA nRGB = colorSensor.getNormalizedColors();
            r += nRGB.red;
            g += nRGB.green;
            b += nRGB.blue;
        }
        return new Vector(r / colorSensors.size(), g / colorSensors.size(), b / colorSensors.size());
    }

    /// An array of RGB readings from all the [NormalizedColorSensor]s as [Vector]s.
    /// @return The readings of each color sensor
    public Vector[] rawRGBs() {
        Vector[] v = new Vector[colorSensors.size()];
        for (int i = 0; i < colorSensors.size(); i++) {
            NormalizedRGBA nRGB = colorSensors.get(i).getNormalizedColors();
            v[i] = new Vector(nRGB.red, nRGB.green, nRGB.blue);
        }
        return v;
    }

    /// Averages the RGB readings from all [NormalizedColorSensor]s and returns the average as a [Vector].
    /// This vector will be normalized between the preset white and black values.
    /// @return The average readings of all color sensors
    public Vector normalizedAvgRGB() {
        return Geometry.normalize(rawAvgRGB(), WHITE, BLACK);
    }

    /// An array of RGB readings from all the [NormalizedColorSensor]s as [Vector]s.
    /// Each vector will be normalized between the preset white and black values.
    /// @return The readings of each color sensor
    public Vector[] normalizedRGBs() {
        Vector[] v = new Vector[colorSensors.size()];
        Vector[] r = rawRGBs();
        for (int i = 0; i < colorSensors.size(); i++) {
            v[i] = Geometry.normalize(r[i], WHITE, BLACK);
        }
        return v;
    }

    /// Finds the name of the closest color to the input color.
    /// The input color is assumed to be a normalized RGB [Vector] between the preset black and white values.
    /// @param normalizedRGB The input color
    /// @param threshold The maximum distance between the input color and any color it is to be compared to
    /// @return The name of the color closest to the input color, or "No Color Within Threshold" if no
    /// colors are within the threshold
    public String closestColor(Vector normalizedRGB, double threshold) {
        double d = threshold;
        String color = "No Color Within Threshold";
        for (String c : colorMap.keySet()) {
            double td = Geometry.subtract(normalizedRGB, colorMap.get(c)).length();
            if (td < d) {
                d = td;
                color = c;
            }
        }
        return color;
    }

    /// Finds the name of the closest color to the input color.
    /// The input color is assumed to be a normalized RGB [Vector] between the preset black and white values.
    /// @param normalizedRGB The input color
    /// @return The name of the color closest to the input color
    public String closestColor(Vector normalizedRGB) {
        return closestColor(normalizedRGB, 2);
    }

    /// Finds the color closest to [#normalizedAvgRGB()].
    /// @return The name of the color closest to the current average reading of the color sensors
    public String closestAverageColor() {
        return closestColor(normalizedAvgRGB());
    }
    /// Finds the color closest to [#normalizedAvgRGB()], within a set `threshold`.
    /// @param threshold The maximum distance between the read color and any color it is to be compared to
    /// @return The name of the color closest to the current average reading of the color sensors
    public String closestAverageColor(double threshold) {
        return closestColor(normalizedAvgRGB(), threshold);
    }

    /// Finds the color closest to any individual [NormalizedColorSensor]'s reading, within a set `threshold`.
    /// @param threshold The maximum distance between the read color and any color it is to be compared to
    /// @return The color closest a reading of a color sensor
    public String closestIndividualColor(double threshold) {
        double d = 2;
        String color = "No Color Within Threshold";
        for (Vector v : normalizedRGBs()) {
            String closest = closestColor(v, threshold);
            double td = Geometry.subtract(colorMap.get(closest), v).length();
            if (td < d) {
                d = td;
                color = closest;
            }
        }
        return color;
    }
    /// Finds the color closest to any individual [NormalizedColorSensor]'s reading.
    /// @return The color closest a reading of a color sensor
    public String closestIndividualColor() {
        return closestIndividualColor(2);
    }
}
