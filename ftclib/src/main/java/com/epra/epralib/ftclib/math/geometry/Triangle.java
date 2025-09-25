package com.epra.epralib.ftclib.math.geometry;
/**Stores a triangle with points a, b, c.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Triangle implements Shape2D{

    private Point a,b,c;
    private Angle angleA, angleB, angleC;

    /**Stores a triangle with points a, b, c.
     * @param a Point a.
     * @param b Point b.
     * @param c Point c.*/
    public Triangle(Point a, Point b, Point c) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.angleA = new Angle();
        this.angleB = new Angle();
        this.angleC = new Angle();
        updateAngles();
    }
    /**Stores a triangle with points a, b, c.
     * @param a Point a.
     * @param b Point b.
     * @param angleA The angle of the two sides intersecting at point a.
     * @param angleB The angle of the two sides intersecting at point b.*/
    public Triangle(Point a, Point b, Angle angleA, Angle angleB) {
        this.a = a;
        this.b = b;
        Angle angleC = Angle.degree(180.0 - (angleA.getDegree() + angleB.getDegree()));
        double d = (Math.sin(angleB.getRadian()) * Geometry.pythagorean(a, b)) / Math.sin(angleC.getRadian());
        this.c = new Point(a.x + (Math.cos(angleA.getRadian()) * d), a.y + (Math.sin(angleA.getRadian()) * d));
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
    public Triangle(Point a, double side1, double side2) {
        this.a = a;
        this.b = new Point(a.x + side1, a.y);
        this.c = new Point(b.x, b.y + side2);
        this.angleA = Angle.radian(Math.atan(side2 / side1));
        this.angleB = Angle.degree(90);
        this.angleC = Angle.radian(Math.atan(side1 / side2));
    }
    /**Stores a triangle with points a, b, c.
     *<p>
     *This constructor creates a right triangle with the angle of the sides intersecting at point b being right.
     * @param a Point a.
     * @param hyp The length of the hypotenuse between point a and point c.
     * @param angleA The angle of the two sides intersecting at point a.
     * */
    public Triangle(Point a, double hyp, Angle angleA) {
        this.a = a;
        this.b = new Point(a.x + (Math.cos(angleA.getRadian()) * hyp), a.y);
        this.c = new Point(b.x, b.y + (Math.sin(angleA.getRadian()) * hyp));
        this.angleA = angleA;
        this.angleB = Angle.degree(90);
        this.angleC = Angle.degree(180.0 - (this.angleA.getDegree() + this.angleB.getDegree()));
    }
    /**Stores a triangle with points a, b, c.
     *<p>
     *This constructor creates a right triangle with the angle of the sides intersecting at point b being right.
     * @param v A vector with a length and an angle.
     * */
    public Triangle(Vector v) {
        this.a = new Point(0.0,0.0);
        this.b = new Point(v.toPoint().x, 0.0);
        this.c = v.toPoint();
        this.angleA = Angle.degree(v.getDegree());
        this.angleB = Angle.degree(90);
        this.angleC = Angle.degree(180.0 - (this.angleA.getDegree() + this.angleB.getDegree()));
    }

    /**@return Point a.*/
    public Point getA() { return a; }
    /**@return Point b.*/
    public Point getB() { return b; }
    /**@return Point c.*/
    public Point getC() { return c; }

    /**@param a Point a.*/
    public void setA(Point a) {
        this.a = a;
        updateAngles();
    }
    /**@param b Point b.*/
    public void setB(Point b) {
        this.b = b;
        updateAngles();
    }
    /**@param c Point c.*/
    public void setC(Point c) {
        this.c = c;
        updateAngles();
    }

    /**@return The length of the side between point a and point b.*/
    public double getAB() { return Geometry.pythagorean(a,b); }
    /**@return The length of the side between point b and point c.*/
    public double getBC() { return Geometry.pythagorean(b,c); }
    /**@return The length of the side between point c and point a.*/
    public double getCA() { return Geometry.pythagorean(c,a); }

    /**@return The angle of the lines intersecting at point a.*/
    public Angle getAngleA() { return angleA; }
    /**@return The angle of the lines intersecting at point b.*/
    public Angle getAngleB() { return angleB; }
    /**@return The angle of the lines intersecting at point c.*/
    public Angle getAngleC() { return angleC; }

    /**@return The area of the triangle.*/
    public double getArea() { return (getBC() * getCA() * Math.sin(angleC.getRadian())) / 2; }

    /**@return The perimeter of the triangle.*/
    public double getPerimeter() { return getAB() + getBC() + getCA(); }

    /**Updates all of the angles to work with the defined points.*/
    private void updateAngles() {
        double aLen = getBC();
        double bLen = getCA();
        double cLen = getAB();
        angleA.setRadian(Math.acos((Math.pow(bLen,2) + Math.pow(cLen,2) - Math.pow(cLen,2)) / 2 * bLen * cLen));
        angleB.setRadian(Math.acos((Math.pow(aLen,2) + Math.pow(cLen,2) - Math.pow(bLen,2)) / 2 * aLen * cLen));
        angleC.setDegree(180.0 - (this.angleA.getDegree() + this.angleB.getDegree()));
    }

    /**@param point Point to check.
     * @return True if the point is within the triangle, false if not.*/
    public boolean checkPoint(Point point) { return (new Triangle(a, b, point).getArea() + new Triangle(b, c, point).getArea() + new Triangle(c, a, point).getArea() == this.getArea()); }
}