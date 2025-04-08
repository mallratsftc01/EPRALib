package epra.math.geometry;
/**Stores a quadrilateral as a construct of two triangles.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Quadrilateral implements Shape2D {

    private Triangle tri1, tri2;
    /**Stores a quadrilateral as a construct of two triangles.
     * @param a Point a.
     * @param b Point b.
     * @param c Point c.
     * @param d Point d.
     * */
    public Quadrilateral(Point a, Point b, Point c, Point d) {
        tri1 = new Triangle(a, b, c);
        tri2 = new Triangle(c, d, a);
    }

    /**
     * @param a Point a.
     * @param b Point b.
     * @param c Point c.
     * @param d Point d.
     */
    public void setPoints (Point a, Point b, Point c, Point d) {
        tri1 = new Triangle(a, b, c);
        tri2 = new Triangle(c, d, a);
    }

    /**@return Point a.*/
    public Point getA() { return tri1.getA(); }
    /**@return Point b.*/
    public Point getB() { return tri1.getB(); }
    /**@return Point c.*/
    public Point getC() { return tri1.getC(); }
    /**@return Point d.*/
    public Point getD() { return tri2.getB(); }

    /**@return The length of the side between point a and point b.*/
    public double getAB() { return tri1.getAB(); }
    /**@return The length of the side between point b and point c.*/
    public double getBC() { return tri1.getBC(); }
    /**@return The length of the side between point c and point d.*/
    public double getCD() { return tri2.getAB(); }
    /**@return The length of the side between point d and point a.*/
    public double getDA() { return tri2.getBC(); }
    /**@return The length of the side between point a and point c.*/
    public double getAC() { return tri1.getCA(); }
    /**@return The length of the side between point b and point d.*/
    public double getBD() { return Geometry.pythagorean(getB(), getD()); }

    /**@return The triangle between points a, b, and c.*/
    public Triangle getABC() { return tri1; }
    /**@return The triangle between points b, c, and d.*/
    public Triangle getBCD() { return new Triangle(getB(), getC(), getD()); }
    /**@return The triangle between points c, d, and a.*/
    public Triangle getCDA() { return tri2; }
    /**@return The triangle between points d, a, and b.*/
    public Triangle getDAB() { return new Triangle(getD(), getA(), getB()); }

    /**@return The perimeter of the quadrilateral.*/
    public double getPerimeter() { return getAB() + getBC() + getCD() + getDA(); }
    /**@return The area of the quadrilateral.*/
    public double getArea() { return tri1.getArea() + tri2.getArea(); }

    /**@param point Point to check.
     * @return True if the point is within the quadrilateral, false if not.*/
    public boolean checkPoint(Point point) { return (tri1.checkPoint(point) || tri2.checkPoint(point)); }
}