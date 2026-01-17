package com.epra.epralib.ftclib.math.geometry;
/// Represents a circle or arc.
///
/// Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Circle implements Shape2D {

    private final Vector center;
    private final double radius;
    private final Angle start;
    private final Angle end;

    /// Represents a full circle.
    /// @param center The center point of the circle
    /// @param radius The radius of the circle
    public Circle(Vector center, double radius) {
        this.center = center;
        this.radius = radius;
        start = new Angle();
        end = new Angle();
    }
    /// Represents an arc.
    ///
    /// The arc is defined between the starting [Angle] and ending [Angle] **counterclockwise**.
    /// @param center The center point of the arc
    /// @param radius The radius of the arc
    /// @param start The starting angle of the arc
    /// @param end The ending angle of the arc
    public Circle(Vector center, double radius, Angle start, Angle end) {
        this.center = center;
        this.radius = radius;
        this.start = start;
        this.end = end;
    }

    /// Returns the center point of the circle or arc as a [Vector].
    /// @return The center point of the circle or arc
    public Vector center() { return center; }
    /// Returns the radius of the circle or arc.
    /// @return The radius of the circle or arc
    public double radius() { return radius; }
    /// Returns the starting [Angle] of the arc, 0 degrees if this represents a full circle.
    /// @return The starting angle of the arc
    public Angle start() { return start; }
    /// Returns the ending [Angle] of the arc, 0 degrees if this represents a full circle.
    /// @return The ending angle of the arc
    public Angle end() { return end; }

    /// Returns the area of the circle or arc.
    /// @return The area of the angle or arc
    public double area() { return Math.PI * Math.pow(this.radius, 2) * (start.degree() == end.degree() ? 1 : Math.abs(end.degree() - start.degree()) / 360.0); }
    /// Returns the circumference if this is a circle or the arc length if this is an arc.
    /// @return The circumference or arc length
    public double circumference() { return Math.PI * this.radius * 2.0 * (start.degree() == end.degree() ? 1 : Math.abs(end.degree() - start.degree()) / 360.0); }

    /// Checks if a given point, represented as a [Vector], is within the circle or arc.
    /// @param point The point to check
    /// @return If the point is within the circle or arc
    public boolean checkPoint(Vector point) {
        if (Geometry.subtract(center, point).length() < radius) {
            return false;
        }
        if (end.degree() > start.degree()) {
            return (point.theta().degree() >= start.degree() && point.theta().degree() <= end.degree());
        } else {
            return (point.theta().degree() <= start.degree() && point.theta().degree() >= end.degree());
        }
    }
}
