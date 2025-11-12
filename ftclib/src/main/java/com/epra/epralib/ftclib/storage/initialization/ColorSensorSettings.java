package com.epra.epralib.ftclib.storage.initialization;

import com.epra.epralib.ftclib.math.geometry.Vector;

import java.util.HashMap;

/**A record that stores settings for color sensors. These settings are the darkest value (black),
 * the lightest value (white), and any other colors normalized between black and white to compare color sensor readings against.
 *<p></p>
 * Queer Coded by Striker-909.*/
public class ColorSensorSettings {

    private final Vector black;
    private final Vector white;
    private final HashMap<String, Vector> colors = new HashMap<>();
    /**A record that stores settings for color sensors.
     * @param black A normalized RGB array representing the darkest color that the color sensors can detect.
     * @param white A normalized RGB array representing the brightest color that the color sensors can detect.
     * @param colors Names paired with RGB arrays normalized relative to the given white and black values representing
     *                  reference colors to compare the color sensor readings against.*/
    public ColorSensorSettings(double[] black, double[] white, HashMap<String, double[]> colors) {
        this.black = new Vector(black[0], black[1], black[2]);
        this.white = new Vector(white[0], white[1], white[2]);
        for (String s : colors.keySet()) {
            this.colors.put(s, new Vector(colors.get(s)[0], colors.get(s)[1], colors.get(s)[2]));
        }
    }

    public Vector black() { return black; }
    public Vector white() { return white; }

    public HashMap<String, Vector> colorMap() { return colors; }
    public String[] colorNames() { return colors.keySet().toArray(new String[0]); }
    public Vector[] colors() { return colors.values().toArray(new Vector[0]); }
}
