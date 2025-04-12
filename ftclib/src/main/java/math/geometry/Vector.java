package math.geometry;

/**Stores an length, theta vector.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Vector extends Angle {

    private double length;

    /**Stores an length, theta vector.
     * @param length
     * @param theta*/
    public Vector(double length, Angle theta) {
        super(theta.getDegree());
        this.length = length;
    }

    /**Stores an length, theta vector.
     * @param x
     * @param y*/
    public Vector(double x, double y) {
        super((float) (Geometry.atan(new Point(y, x)).getRadian()));
        this.length = Geometry.pythagorean(x, y);
    }

    /**Stores an length, theta vector.
     * @param point Point at the end of the vector.*/
    public Vector(Point point) {
        super((float) (Geometry.atan(Geometry.reverse(point)).getRadian()));
        this.length = Geometry.pythagorean(point.x, point.y);
    }

    /**@return The length of the vector.*/
    public double getLength() { return length; }
    /**@param length The new length of the vector.*/
    public void setLength(double length) { this.length = length; }

    /**@param point Point at the end of the vector.*/
    public void setPoint(Point point) {
        Angle radian = Geometry.atan(Geometry.reverse(point));
        super.setDegree(radian.getDegree());
        this.length = Geometry.pythagorean(point.x, point.y);
    }

    /**@return The vector converted to an x, y coordinate point/*/
    public Point toPoint() { return new Point(Math.cos(super.getRadian()) * length, Math.sin(super.getRadian()) * length); }
    /**@return A right triangle between 0,0, vector x,0, and vector x, vector y.*/
    public Triangle toTriangle() { return new Triangle(this); }
}
