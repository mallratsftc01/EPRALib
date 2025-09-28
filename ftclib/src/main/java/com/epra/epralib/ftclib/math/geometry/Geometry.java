package com.epra.epralib.ftclib.math.geometry;

import org.opencv.core.Mat;

/**A class that adds many geometric functions for many uses.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Geometry {

    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The resulting angle from adding the first to the second, in a range between 0 and 360 degrees (0 to 2pi).*/
    public static Angle add(Angle angle1, Angle angle2) { return Angle.degree((angle1.degree() + angle2.degree()) % 360); }
    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The resulting angle from subtracting the second from the first, in a range between 0 and 360 degrees (0 to 2pi).*/
    public static Angle subtract(Angle angle1, Angle angle2) { return Angle.degree((angle1.degree() - angle2.degree()) % 360); }
    /**@param angle Array of angles.
     * @return The average angle of the array.*/
    public static Angle average(Angle[] angle) {
        double degrees = 0.0;
        for (Angle a : angle) { degrees += a.degree(); }
        degrees /= angle.length;
        return Angle.degree(degrees % 360.0);
    }

    /**@param angle An angle to scale.
     * @param scalar A scalar to scale the angle by.
     * @return The angle scaled by the scalar.*/
    public static Angle scale(Angle angle, double scalar) { return Angle.degree(angle.degree() * scalar % 360); }

    /**@param angle An angle.
     * @return The trigonometric sine value of an angle.*/
    public static double sin(Angle angle) { return Math.sin(angle.radian()); }
    /**@param angle An angle.
     * @return The trigonometric cosine value of an angle.*/
    public static double cos(Angle angle) { return Math.cos(angle.radian()); }
    /**@param angle An angle.
     * @return The trigonometric tangent value of an angle.*/
    public static double tan(Angle angle) { return Math.tan(angle.radian()); }
    /**@param angle An angle.
     * @return The trigonometric cosecant value of an angle.*/
    public static double csc(Angle angle) { return 1.0 / Math.sin(angle.radian()); }
    /**@param angle An angle.
     * @return The trigonometric secant value of an angle.*/
    public static double sec(Angle angle) { return 1.0 / Math.cos(angle.radian()); }
    /**@param angle An angle.
     * @return The trigonometric cotangent value of an angle.*/
    public static double cot(Angle angle) { return 1.0 / Math.tan(angle.radian()); }
    /**@param a The value whose arc sine is to be returned.
     * @return The arc sine of a value; the returned angle is in the range -pi/2 to pi/2.*/
    public static Angle asin(double a) { return Angle.radian(Math.asin(a)); }
    /**@param a The value whose arc cosine is to be returned.
     * @return The arc cosine of a value; the returned angle is in the range 0.0 to pi.*/
    public static Angle acos(double a) { return Angle.radian(Math.acos(a)); }
    /**@param a The value whose arc tangent is to be returned.
     * @return The arc tangent of a value; the returned angle is in the range -pi/2 to pi/2.*/
    public static Angle atan(double a) { return Angle.radian(Math.atan(a)); }
    /**@param x The x coordinate.
     * @param y The y coordinate.
     * @return The angle of the 2D vector that ends at the Vector (x,y)*/
    public static Angle atan(double x, double y) { return Angle.radian(Math.atan2(y, x)); }

    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return The resulting vector from adding the first to the second.*/
    public static Vector add(Vector vector1, Vector vector2) { return new Vector(vector1.x() + vector2.x(), vector1.y() + vector2.y(), vector1.z() + vector2.z()); }
    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return The resulting vector from subtracting the second from the first.*/
    public static Vector subtract(Vector vector1, Vector vector2) { return new Vector(vector2, vector1); }
    /**Scales a vector by a scalar.
     * @param vector A vector.
     * @param scalar A double scalar.
     * @return The vector scaled by the scalar.*/
    public static Vector scale(Vector vector, double scalar) { return new Vector(vector.x() * scalar, vector.y() * scalar, vector.z() * scalar); }
    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return Returns the dot product of the two vectors.*/
    public static double dot(Vector vector1, Vector vector2) { return (vector1.x() * vector2.x()) + (vector1.y() * vector2.y()) + (vector1.z() * vector2.z()); }

    /**@param matrix1 First matrix.
     * @param matrix2 Second matrix.
     * @return The resulting matrix from adding the first to the second.*/
    public static Matrix add(Matrix matrix1, Matrix matrix2) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix1.get(i, j) + matrix2.get(i, j);
            }
        }
        return new Matrix(temp);
    }
    /**@param matrix1 First matrix.
     * @param matrix2 Second matrix.
     * @return The resulting matrix from subtracting the first from the second.*/
    public static Matrix subtract(Matrix matrix1, Matrix matrix2) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix1.get(i, j) - matrix2.get(i, j);
            }
        }
        return new Matrix(temp);
    }
    /**@param matrix A matrix.
     * @param scalar A scalar to scale the matrix by.
     * @return The resulting matrix from scaling every element of the matrix by the scalar.*/
    public static Matrix scale(Matrix matrix, double scalar) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix.get(i, j) * scalar;
            }
        }
        return new Matrix(temp);
    }

    /**@param matrix1 First matrix.
     * @param matrix2 Second matrix.
     * @return The matrix resulting from multiplying the first by the second.*/
    public static Matrix multiply(Matrix matrix1, Matrix matrix2) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix1.get(i, 0) * matrix2.get(0, j) +
                        matrix1.get(i, 1) * matrix2.get(1, j) +
                        matrix1.get(i, 2) * matrix2.get(2, j);
            }
        }
        return new Matrix(temp);
    }
    /**@param matrix A matrix.
     * @param vector A vector.
     * @return The vector resulting from multiplying the matrix by the vector.*/
    public static Vector multiply(Matrix matrix, Vector vector) {
        double x = matrix.get(0, 0) * vector.x() + matrix.get(0, 1) * vector.y() + matrix.get(0, 2) * vector.z();
        double y = matrix.get(1, 0) * vector.x() + matrix.get(1, 1) * vector.y() + matrix.get(1, 2) * vector.z();
        double z = matrix.get(2, 0) * vector.x() + matrix.get(2, 1) * vector.y() + matrix.get(2, 2) * vector.z();
        return new Vector(x, y, z);
    }

    /**@param a Length of leg a.
     * @param b Length of leg b.
     * @return The distance from the origin to a Vector a, b.*/
    public static double pythagorean(double a, double b) { return Math.sqrt((a * a) + (b * b));}

    /**@param a The x coordinate.
     * @param b The y coordinate.
     * @param c The z coordinate.
     * @return The distance from the origin to a Vector a, b, c.*/
    public static double pythagorean(double a, double b, double c) { return Math.sqrt((a * a) + (b * b) + (c * c));}

    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The direction for the first angle to reach the second angle the quickest (1.0 is clockwise, -1.0 is counterclockwise).*/
    public static double direction(Angle angle1, Angle angle2) {
        double sign = Math.signum(angle1.degree() - angle2.degree());
        if (Math.max(angle1.degree() % 360, angle2.degree() % 360) > 270 && Math.min(angle1.degree() % 360, angle2.degree() % 360) < 90) {
            sign *= -1;
        }
        return sign;
    }
}