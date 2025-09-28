package com.epra.epralib.ftclib.math.geometry;
/**Stores a full or partial circle.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Circle implements Shape2D {

    private Vector center;
    private double radius;
    private Angle start;
    private Angle end;

    /**Stores a full circle.
     * @param center The center Vector of the circle.
     * @param radius The radius of the circle.*/
    public Circle(Vector center, double radius) {
        this.center = center;
        this.radius = radius;
        start = new Angle();
        end = new Angle();
    }
    /**Stores a full circle or partial circle. The partial circle is defined between the start angle and end angle COUNTERCLOCKWISE.
     *<p></p>
     * @param center The center Vector of the circle.
     * @param radius The radius of the circle.
     * @param start The start angle of the circle.
     * @param end The end angle of the circle.*/
    public Circle(Vector center, double radius, Angle start, Angle end) {
        this.center = center;
        this.radius = radius;
        this.start = start;
        this.end = end;
    }

    /**@return The center Vector of the circle.*/
    public Vector getCenter() { return center; }
    /**@return The radius of the circle.*/
    public double getRadius() { return radius; }
    /**@return The start angle of the circle.*/
    public Angle getStart() { return start; }
    /**@return The end angle of the circle.*/
    public Angle getEnd() { return end; }

    /**@return The area of the circle.*/
    public double getArea() { return Math.PI * Math.pow(this.radius, 2); }
    /**@return The circumference of the circle.*/
    public double getCircumference() { return Math.PI * this.radius * 2.0; }

    /**@param point Point to check.
     * @return True if the Vector is within the circle, false if not.*/
    public boolean checkPoint(Vector point) {
        if (end.degree() > start.degree()) {
            return (Geometry.subtract(center, point).length() >= radius && point.theta().degree() >= start.degree() && point.theta().degree() <= end.degree());
        } else {
            return (Geometry.subtract(center, point).length() >= radius && point.theta().degree() <= start.degree() && point.theta().degree() >= end.degree());
        }
    }
}
