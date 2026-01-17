package com.epra.epralib.ftclib.math.geometry;
/**Stores a triangle with Vectors a, b, c.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Triangle implements Shape2D{

    private final Vector a,b,c;
    private final Angle angleA, angleB, angleC;

    /**Stores a triangle with points a, b, c.
     * @param a Point a.
     * @param b Point b.
     * @param c Point c.*/
    public Triangle(Vector a, Vector b, Vector c) {
        this.a = a;
        this.b = b;
        this.c = c;
        double side1 = getAB();
        double side2 = getBC();
        this.angleA = Geometry.atan(side2, side1);
        this.angleB = Angle.degree(90);
        this.angleC = Geometry.atan(side1, side2);
    }
    /**Stores a triangle with points a, b, c.
     * @param a Point a.
     * @param b Point b.
     * @param angleA The angle of the two sides intersecting at point a.
     * @param angleB The angle of the two sides intersecting at point b.*/
    public Triangle(Vector a, Vector b, Angle angleA, Angle angleB) {
        this.a = a;
        this.b = b;
        Angle angleC = Angle.degree(180.0 - (angleA.degree() + angleB.degree()));
        double d = (Geometry.sin(angleB) * Geometry.subtract(a, b).length()) / Geometry.sin(angleC);
        this.c = new Vector(a.x() + (Geometry.cos(angleA) * d), a.y()+ (Geometry.sin(angleA) * d));
        this.angleA = angleA;
        this.angleB = angleB;
        this.angleC = angleC;
    }

    //Right Triangles
    /**Stores a triangle with points a, b, c.
     *<p>
     *This constructor creates a right triangle with the angle of the sides intersecting at point b being right.
     * @param a Point a.
     * @param side1 The length of the side between point a and point b.
     * @param side2 The length of the side between point b and point c.*/
    public Triangle(Vector a, double side1, double side2) {
        this.a = a;
        this.b = new Vector(a.x() + side1, a.y());
        this.c = new Vector(b.x(), b.y() + side2);
        this.angleA = Geometry.atan(side2, side1);
        this.angleB = Angle.degree(90);
        this.angleC = Geometry.atan(side1, side2);
    }
    /**Stores a triangle with points a, b, c.
     *<p>
     *This constructor creates a right triangle with the angle of the sides intersecting at point b being right.
     * @param a Point a.
     * @param hyp The length of the hypotenuse between point a and point c.
     * @param angleA The angle of the two sides intersecting at point a.
     * */
    public Triangle(Vector a, double hyp, Angle angleA) {
        this.a = a;
        this.b = new Vector(a.x() + (Geometry.cos(angleA) * hyp), a.y());
        this.c = new Vector(b.x(), b.y() + (Geometry.sin(angleA) * hyp));
        this.angleA = angleA;
        this.angleB = Angle.degree(90);
        this.angleC = Angle.degree(180.0 - (this.angleA.degree() + this.angleB.degree()));
    }
    /**Stores a triangle with points a, b, c.
     *<p>
     *This constructor creates a right triangle with the angle of the sides intersecting at point b being right.
     * @param v A vector with a length and an angle.
     * */
    public Triangle(Vector v) {
        this.a = new Vector(0.0,0.0);
        this.b = new Vector(v.x(), 0.0);
        this.c = v;
        this.angleA = v.theta();
        this.angleB = Angle.degree(90);
        this.angleC = Angle.degree(180.0 - (this.angleA.degree() + this.angleB.degree()));
    }

    /**@return Point a.*/
    public Vector getA() { return a; }
    /**@return Point b.*/
    public Vector getB() { return b; }
    /**@return Point c.*/
    public Vector getC() { return c; }

    /**@return The length of the side between Vector a and Vector b.*/
    public double getAB() { return Geometry.subtract(a,b).length(); }
    /**@return The length of the side between Vector b and Vector c.*/
    public double getBC() { return Geometry.subtract(b,c).length(); }
    /**@return The length of the side between Vector c and Vector a.*/
    public double getCA() { return Geometry.subtract(c,a).length(); }

    /**@return The angle of the lines intersecting at Vector a.*/
    public Angle getAngleA() { return angleA; }
    /**@return The angle of the lines intersecting at Vector b.*/
    public Angle getAngleB() { return angleB; }
    /**@return The angle of the lines intersecting at Vector c.*/
    public Angle getAngleC() { return angleC; }

    /**@return The area of the triangle.*/
    public double area() { return (getBC() * getCA() * Math.sin(angleC.radian())) / 2; }

    /**@return The perimeter of the triangle.*/
    public double getPerimeter() { return getAB() + getBC() + getCA(); }

    /**@param point Point to check.
     * @return True if the Vector is within the triangle, false if not.*/
    public boolean checkPoint(Vector point) { return (new Triangle(a, b, point).area() + new Triangle(b, c, point).area() + new Triangle(c, a, point).area() == this.area()); }
}