package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.Vector;

import com.epra.epralib.ftclib.math.geometry.Angle;

/// Stores a pose value with a [Vector] position and an [Angle].
///
/// Represents a specific position and rotation on the field.
/// Mainly used by [Odometry].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class Pose {

    public Vector pos;
    public Angle angle;

    /// Stores a pose value with a [Vector] position and an [Angle].
    ///
    /// Represents a specific position and rotation on the field.
    /// Mainly used by [Odometry].
    /// @param pos The vector position to store
    /// @param angle The angle to store
    public Pose(Vector pos, Angle angle) {
        this.pos = pos;
        this.angle = angle;
    }
}
