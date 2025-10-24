package com.epra.epralib.ftclib.math.physics;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Geometry;
import com.epra.epralib.ftclib.math.geometry.Vector;

public class ProjectileMotion {

    public static final Vector GRAVITY = new Vector(0, 0, -9.81);

    public static double projectileInitialVelocity(Vector start, Angle launchAngle, Vector target) {
        Vector delta = new Vector(start, target);
        double deltaZ = target.z() - start.z();
        double deltaXY = Math.sqrt(delta.length() * delta.length() - deltaZ * deltaZ);
        return Math.sqrt((GRAVITY.length() * deltaXY * deltaXY) /
                (deltaXY * Geometry.sin(Geometry.add(launchAngle, launchAngle)))
                - (2 * deltaZ * Math.pow(Geometry.cos(launchAngle), 2)));
    }
}
