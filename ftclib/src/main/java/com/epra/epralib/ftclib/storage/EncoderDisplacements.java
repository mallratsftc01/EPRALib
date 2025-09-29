package com.epra.epralib.ftclib.storage;

import com.epra.epralib.ftclib.math.geometry.Vector;

/**A record that stores the displacements of encoders from the center of the robot in inches for odometry.
 *<p></p>
 * Queer Coded by Striker-909.*/
public record EncoderDisplacements(double leftX, double leftY, double rightX, double rightY, double perpendicularX, double perpendicularY) {
    public Vector left() { return new Vector(leftX, leftY); }
    public Vector right() { return new Vector(rightX, rightY); }
    public Vector perpendicular() { return new Vector(perpendicularX, perpendicularY); }
}
