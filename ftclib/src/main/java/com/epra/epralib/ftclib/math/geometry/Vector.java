package com.epra.epralib.ftclib.math.geometry;

/**Stores a three-dimensional vector.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Vector {

    public static final Vector ZERO = new Vector(0, 0, 0);
    public static final Vector I_HAT = new Vector(1, 0, 0);
    public static final Vector J_HAT = new Vector(0, 1, 0);
    public static final Vector K_HAT = new Vector(0, 0, 1);

    private double length;
    private Angle theta = new Angle(); //azimuthal angle (rotation around z)
    private Angle phi = new Angle(); //polar angle, from the x,y plane
    private double x, y, z;
    private Vector(double length, Angle theta, Angle phi, double x, double y, double z) {
        this.length = length;
        this.theta = theta;
        this.phi = phi;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**Stores a vector with z and phi set to 0.
     * @param length The length of the vector.
     * @param theta The angle of the vector around the z axis.*/
    public Vector(double length, Angle theta) {
        this(length, theta, new Angle(), length * Geometry.cos(theta), length * Geometry.sin(theta), 0);
    }

    /**Stores a vector.
     * @param length The length of the vector.
     * @param theta The angle of the vector around the z axis.
     * @param phi The angle of the vector from the xy plane. (90 degrees is vertical upwards, -90 degrees is vertical downwards).*/
    public Vector(double length, Angle theta, Angle phi) {
        this(length, theta, phi, length * Geometry.cos(theta) * Geometry.cos(phi), length * Geometry.sin(theta) * Geometry.cos(phi), length * Geometry.sin(phi));
    }

    /**Stores a vector with z and phi set to 0.
     * @param x The x coordinate.
     * @param y The y coordinate.*/
    public Vector(double x, double y) {
        this(Geometry.pythagorean(x, y), Geometry.atan(x, y), new Angle(), x, y, 0);
    }

    /**Stores a vector.
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @param z The z coordinate.*/
    public Vector(double x, double y, double z) {
        this.length = Geometry.pythagorean(x, y, z);
        this.theta = Geometry.atan(x, y);
        this.phi = Geometry.subtract(Angle.degree(90), Geometry.acos(z / length));
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**Stores a vector with a translated origin.
     * @param origin A vector representing the origin of the vector.
     * @param end A vector representing the end of the vector.*/
    public Vector(Vector origin, Vector end) {
        this(end.x() - origin.x(), end.y() - origin.y(), end.z() - origin.z());
    }

    /**@return The length of the vector.*/
    public double length() { return length; }
    /**@return The angle of the vector around the z axis.*/
    public Angle theta() { return theta; }
    /**@return The angle of the vector from the xy plane. (90 degrees is vertical upwards, -90 degrees is vertical downwards).*/
    public Angle phi() { return phi; }
    /**@return The x coordinate of the vector.*/
    public double x() { return x; }
    /**@return The y coordinate of the vector.*/
    public double y() { return y; }
    /**@return The z coordinate of the vector.*/
    public double z() { return z; }

    /**@return The vector written in (x,y,z) form.*/
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    public boolean equals(Vector v) {
        return (x == v.x() && y == v.y() && z == v.z());
    }

    public int hashCode() {
        return toString().hashCode();
    }
}
