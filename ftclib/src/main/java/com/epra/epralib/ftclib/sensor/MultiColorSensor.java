package com.epra.epralib.ftclib.sensor;

import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.storage.initialization.ColorSensorSettings;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * Increases the functionality of the color sensor and can combine data from multiple color sensors.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class MultiColorSensor {

    private final ArrayList<NormalizedColorSensor> colorSensors = new ArrayList<>();
    private final Vector BLACK;
    private final Vector WHITE;
    private final HashMap<String, Vector> colorMap = new HashMap<>();

    /**Increases the functionality of the color sensor and can combine data from multiple color sensors.
     * @param blackRGB A normalized RGB vector representing the darkest color that the color sensors can detect.
     * @param whiteRGB A normalized RGB vector representing the brightest color that the color sensors can detect.
     * @param colorNames The names of reference colors to compare color sensor readings against.
     * @param colorsRGB RGB vectors normalized relative to the given white and black values representing
     *                 reference colors to compare the color sensor readings against.
     * @param colorSensor A color sensor.
     * @param colorSensors Any number of additional color sensors.*/
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
    /**Increases the functionality of the color sensor and can combine data from multiple color sensors.
     * @param blackRGB A normalized RGB vector representing the darkest color that the color sensors can detect.
     * @param whiteRGB A normalized RGB vector representing the brightest color that the color sensors can detect.
     * @param colorSensor A color sensor.
     * @param colorSensors Any number of additional color sensors.*/
    public MultiColorSensor(Vector blackRGB, Vector whiteRGB, NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(blackRGB, whiteRGB, new String[0], new Vector[0], colorSensor, colorSensors);
    }
    /**Increases the functionality of the color sensor and can combine data from multiple color sensors.
     * @param colorSensor A color sensor.
     * @param colorSensors Any number of additional color sensors.*/
    public MultiColorSensor(NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(new Vector(0, 0, 0), new Vector(1, 1, 1), colorSensor, colorSensors);
    }
    /**Increases the functionality of the color sensor and can combine data from multiple color sensors.
     * @param colorSensorSettings An object containing settings for this set of color sensors
     * @param colorSensor A color sensor.
     * @param colorSensors Any number of additional color sensors.*/
    public MultiColorSensor(ColorSensorSettings colorSensorSettings, NormalizedColorSensor colorSensor, NormalizedColorSensor... colorSensors) {
        this(colorSensorSettings.black(), colorSensorSettings.white(),
                colorSensorSettings.colorNames(), colorSensorSettings.colors(), colorSensor, colorSensors);
    }

    /**@return The average RGB readings of all color sensors, as a Vector.*/
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

    /**@return The RGB readings of each color sensor as an array of Vectors.*/
    public Vector[] rawRGBs() {
        Vector[] v = new Vector[colorSensors.size()];
        for (int i = 0; i < colorSensors.size(); i++) {
            NormalizedRGBA nRGB = colorSensors.get(i).getNormalizedColors();
            v[i] = new Vector(nRGB.red, nRGB.green, nRGB.blue);
        }
        return v;
    }

    /**@return The average RGB readings of all color sensors, as a Vector normalized between the preset white and black values.*/
    public Vector normalizedAvgRGB() {
        return Geometry.normalize(rawAvgRGB(), WHITE, BLACK);
    }

    /**@return The RGB readings of each color sensor as an array of Vectors normalized between the preset white and black values.*/
    public Vector[] normalizedRGBs() {
        Vector[] v = new Vector[colorSensors.size()];
        Vector[] r = rawRGBs();
        for (int i = 0; i < colorSensors.size(); i++) {
            v[i] = Geometry.normalize(r[i], WHITE, BLACK);
        }
        return v;
    }

    /**Gives the name of the closest color to the input color.
     * The input color is assumed to be normalized between the preset black and white values.
     * @param normalizedRGB The input color as a normalized Vector.
     * @param threshold The maximum distance between the input color and any color it is to be compared to.
     * @return The name of the color closest to the input color, or "No Color Within Threshold" if no
     * colors are within the threshold.*/
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

    /**Gives the name of the closest color to the input color.
     * The input color is assumed to be normalized between the preset black and white values.
     * @param normalizedRGB The input color as a normalized Vector.
     * @return The name of the color closest to the input color.*/
    public String closestColor(Vector normalizedRGB) {
        return closestColor(normalizedRGB, 2);
    }

    /**@return The name of the color closest to the current average reading of the color sensors.*/
    public String closestAverageColor() {
        return closestColor(normalizedAvgRGB());
    }
    /**@param threshold The maximum distance between the read color and any color it is to be compared to.
     * @return The name of the color closest to the current average reading of the color sensors.*/
    public String closestAverageColor(double threshold) {
        return closestColor(normalizedAvgRGB(), threshold);
    }

    /**@param threshold The maximum distance between the read color and any color it is to be compared to.
     * @return The color closest a reading of a color sensor.*/
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
    /**@return The color closest a reading of a color sensor.*/
    public String closestIndividualColor() {
        return closestIndividualColor(2);
    }
}
