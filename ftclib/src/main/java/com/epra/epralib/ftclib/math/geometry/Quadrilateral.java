package com.epra.epralib.ftclib.math.geometry;
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
    public Quadrilateral(Vector a, Vector b, Vector c, Vector d) {
        tri1 = new Triangle(a, b, c);
        tri2 = new Triangle(c, d, a);
    }

    /**@return Vector a.*/
    public Vector getA() { return tri1.getA(); }
    /**@return Vector b.*/
    public Vector getB() { return tri1.getB(); }
    /**@return Vector c.*/
    public Vector getC() { return tri1.getC(); }
    /**@return Vector d.*/
    public Vector getD() { return tri2.getB(); }

    /**@return The length of the side between Vector a and Vector b.*/
    public double getAB() { return tri1.getAB(); }
    /**@return The length of the side between Vector b and Vector c.*/
    public double getBC() { return tri1.getBC(); }
    /**@return The length of the side between Vector c and Vector d.*/
    public double getCD() { return tri2.getAB(); }
    /**@return The length of the side between Vector d and Vector a.*/
    public double getDA() { return tri2.getBC(); }
    /**@return The length of the side between Vector a and Vector c.*/
    public double getAC() { return tri1.getCA(); }
    /**@return The length of the side between Vector b and Vector d.*/
    public double getBD() { return Geometry.subtract(getB(), getD()).length(); }

    /**@return The triangle between Vectors a, b, and c.*/
    public Triangle getABC() { return tri1; }
    /**@return The triangle between Vectors b, c, and d.*/
    public Triangle getBCD() { return new Triangle(getB(), getC(), getD()); }
    /**@return The triangle between Vectors c, d, and a.*/
    public Triangle getCDA() { return tri2; }
    /**@return The triangle between Vectors d, a, and b.*/
    public Triangle getDAB() { return new Triangle(getD(), getA(), getB()); }

    /**@return The perimeter of the quadrilateral.*/
    public double getPerimeter() { return getAB() + getBC() + getCD() + getDA(); }
    /**@return The area of the quadrilateral.*/
    public double getArea() { return tri1.getArea() + tri2.getArea(); }

    /**@param point Point to check.
     * @return True if the Vector is within the quadrilateral, false if not.*/
    public boolean checkPoint(Vector point) { return (tri1.checkPoint(point) || tri2.checkPoint(point)); }
}