package epra.math.geometry;
/**Stores a full or partial circle.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Circle implements Shape2D {

    private Point center;
    private double radius;
    private Angle start;
    private Angle end;

    /**Stores a full circle.
     * @param center The center point of the circle.
     * @param radius The radius of the circle.*/
    public Circle(Point center, double radius) {
        this.center = center;
        this.radius = radius;
        start = new Angle(0.0);
        end = new Angle(360.0);
    }
    /**Stores a full circle or partial circle. The partial circle is defined between the start angle and end angle COUNTERCLOCKWISE.
     *<p></p>
     * @param center The center point of the circle.
     * @param radius The radius of the circle.
     * @param start The start angle of the circle.
     * @param end The end angle of the circle.*/
    public Circle(Point center, double radius, Angle start, Angle end) {
        this.center = center;
        this.radius = radius;
        this.start = start;
        this.end = end;
    }

    /**@param center The center point of the circle.*/
    public void setCenter(Point center) { this.center = center; }
    /**@param radius The radius of the circle.*/
    public void setRadius(double radius) { this.radius = radius; }
    /**@param start The start angle of the circle.*/
    public void setStart(Angle start) { this.start = start; }
    /**@param end The end angle of the circle.*/
    public void setEnd(Angle end) { this.end = end; }


    /**@return The center point of the circle.*/
    public Point getCenter() { return center; }
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
     * @return True if the point is within the circle, false if not.*/
    public boolean checkPoint(Point point) {
        Angle angle = Geometry.atan(point);
        if (end.getDegree() > start.getDegree()) {
            return (Geometry.pythagorean(center, point) >= radius && angle.getDegree() >= start.getDegree() && angle.getDegree() <= end.getDegree());
        } else {
            return (Geometry.pythagorean(center, point) >= radius && angle.getDegree() <= start.getDegree() && angle.getDegree() >= end.getDegree());
        }
    }
}
