package com.epra.epralib.ftclib.math.geometry;

/**Stores an Angle value.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Angle {

    /**Represents an angle of pi radians or 180.0 degrees.*/
    public static final Angle PI = Angle.radian(Math.PI);
    /**Represents an angle of 1.0 radian or ~57.296 degrees.*/
    public static final Angle RAD = Angle.radian(1);

    private double radian;
    private double degree;

    /**Stores an Angle value.
     *<p></p>
     * @param radian Radian value to set stored value to.*/
    public static Angle radian(double radian) { return new Angle((float)radian); }
    /**Stores an Angle value.
     *<p></p>
     * @param degree Degree value to set stored value to.*/
    public static Angle degree(double degree) { return new Angle(degree); }

    /**Stores an Angle value.
     *<p></p>
     * @param radian Radian value to set stored value to.*/
    private Angle(float radian) {
        this.radian = radian;
        this.degree = (180.0 / Math.PI) * radian;
    }
    /**Stores an Angle value.
     *<p></p>
     * @param degree Degree value to set stored value to.*/
    private Angle(double degree) {
        this.radian = (Math.PI / 180.0) * degree;
        this.degree = degree;
    }
    /**Stores an Angle value.*/
    public Angle() { this.radian = 0.0; }

    /**@return The radian stored value;*/
    public double radian() { return this.radian;}
    /**@return The degree stored value;*/
    public double degree() { return this.degree; }
}