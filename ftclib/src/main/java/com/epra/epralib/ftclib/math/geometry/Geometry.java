package com.epra.epralib.ftclib.math.geometry;

/// A class that adds many geometric functions for many uses.
///
/// Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.
public class Geometry {

    /// Adds two [Angles](Angle).
    ///
    /// The resulting [Angle] will be in the range 0 and 360 degrees (0 to 2π radians).
    /// @param angle1 First angle
    /// @param angle2 Second angle
    /// @return The sum of the two angles
    /// in a range between 0 and 360 degrees (0 to 2π)
    public static Angle add(Angle angle1, Angle angle2) { return Angle.degree((angle1.degree() + angle2.degree()) % 360); }
    /// Subtracts one [Angle] from another.
    ///
    /// The resulting [Angle] will be in the range 0 and 360 degrees (0 to 2π radians).
    /// @param angle1 First angle
    /// @param angle2 Second angle
    /// @return The second angle subtracted from the first
    public static Angle subtract(Angle angle1, Angle angle2) { return Angle.degree((angle1.degree() - angle2.degree()) % 360); }
    /// Finds the average [Angle] from an array of [Angle]
    /// @param angles An array of angles
    /// @return The average angle of the array
    public static Angle average(Angle[] angles) {
        double degrees = 0.0;
        for (Angle a : angles) { degrees += a.degree(); }
        degrees /= angles.length;
        return Angle.degree(degrees % 360.0);
    }

    /// Scales an [Angle] by a scalar.
    ///
    /// The resulting [Angle] will be in the range 0 and 360 degrees (0 to 2π radians).
    /// @param angle The angle to scale
    /// @param scalar A scalar to scale the angle by
    /// @return The angle scaled by the scalar
    public static Angle scale(Angle angle, double scalar) { return Angle.degree(angle.degree() *  scalar % 360); }

    /// Reflects an [Angle] over the x-axis.
    ///
    /// The resulting [Angle] will be in the range 0 and 360 degrees (0 to 2π radians).
    /// @param angle The angle to reflect
    /// @return The angle reflected over the x-axis
    public static Angle reflectX(Angle angle) { return Angle.degree((180 - angle.degree()) % 360); }
    /// Reflects an [Angle] over the y-axis.
    ///
    /// The resulting [Angle] will be in the range 0 and 360 degrees (0 to 2π radians).
    /// @param angle The angle to reflect
    /// @return The angle reflected over the y-axis
    public static Angle reflectY(Angle angle) { return Angle.degree((-1 * angle.degree()) % 360); }

    /// Finds the trigonometric sine value of an [Angle].
    ///
    /// The result will be in a range from -1 to 1.
    /// @param angle An angle
    /// @return The trigonometric sine value of the angle
    public static double sin(Angle angle) { return Math.sin(angle.radian()); }
    /// Finds the trigonometric cosine value of an [Angle].
    ///
    /// The result will be in a range from -1 to 1.
    /// @param angle An angle
    /// @return The trigonometric cosine value of the angle
    public static double cos(Angle angle) { return Math.cos(angle.radian()); }
    /// Finds the trigonometric tangent value of an [Angle].
    ///
    /// The result will be in a range from -∞ to ∞.
    /// @param angle An angle
    /// @return The trigonometric tangent value of the angle
    public static double tan(Angle angle) { return Math.tan(angle.radian()); }
    /// Finds the trigonometric cosecant value of an [Angle].
    ///
    /// The result will be in a range from -∞ to -1 and 1 to ∞.
    /// @param angle An angle
    /// @return The trigonometric cosecant value of the angle
    public static double csc(Angle angle) { return 1.0 / Math.sin(angle.radian()); }
    /// Finds the trigonometric secant value of an [Angle].
    ///
    /// The result will be in a range from -∞ tp -1 and 1 to ∞.
    /// @param angle An angle
    /// @return The trigonometric secant value of the angle
    public static double sec(Angle angle) { return 1.0 / Math.cos(angle.radian()); }
    /// Finds the trigonometric cotangent value of an [Angle].
    ///
    /// The result will be in a range from -∞ to ∞.
    /// @param angle An angle
    /// @return The trigonometric sine value of the angle
    public static double cot(Angle angle) { return 1.0 / Math.tan(angle.radian()); }
    /// Finds the trigonometric arc sine [Angle] corresponding to a value.
    ///
    /// The result will be in a range from -π/2 to π/2.
    /// @param a A value
    /// @return The trigonometric arc sine angle corresponding to the value
    public static Angle asin(double a) { return Angle.radian(Math.asin(a)); }
    /// Finds the trigonometric arc cosine [Angle] corresponding to a value.
    ///
    /// The result will be in a range from 0 to π.
    /// @param a A value
    /// @return The trigonometric arc cosine angle corresponding to the value
    public static Angle acos(double a) { return Angle.radian(Math.acos(a)); }
    /// Finds the trigonometric arc tangent [Angle] corresponding to a value.
    ///
    /// The result will be in a range from -π/2 to π/2.
    /// @param a A value
    /// @return The trigonometric arc tangent angle corresponding to the value
    public static Angle atan(double a) { return Angle.radian(Math.atan(a)); }
    /// Finds the trigonometric arc tangent [Angle] of a [Vector] ending at `(x, y)`,
    /// the angle between that [Vector] and the x-axis.
    ///
    /// The result will be in a range from -π to π.
    /// @param x The x coordinate of the vector
    /// @param y The y coordinate of the vector
    /// @return The angle between the vector and the x-axis
    public static Angle atan(double x, double y) { return Angle.radian(Math.atan2(y, x)); }

    /// Adds two [Vectors](Vector).
    /// @param vector1 First vector
    /// @param vector2 Second vector
    /// @return The sum of the two vectors
    public static Vector add(Vector vector1, Vector vector2) { return new Vector(vector1.x() + vector2.x(), vector1.y() + vector2.y(), vector1.z() + vector2.z()); }
    /// Subtracts one [Vector] from another
    /// @param vector1 First vector
    /// @param vector2 Second vector
    /// @return The second vector subtracted from the first
    public static Vector subtract(Vector vector1, Vector vector2) { return new Vector(vector2, vector1); }
    /// Scales a [Vector] by a scalar
    /// @param vector A vector
    /// @param scalar A scalar
    /// @return The vector scaled by the scalar
    public static Vector scale(Vector vector, double scalar) { return new Vector(vector.x() * scalar, vector.y() * scalar, vector.z() * scalar); }
    /// Computes the hadamard-product of two [Vectors](Vector), producing a [Vector].
    ///
    /// [Wikipedia Hadamard Product](https://en.wikipedia.org/wiki/Hadamard_product_(matrices))
    /// @param vector1 First vector
    /// @param vector2 Second vector
    /// @return The hadamard-product of the two vectors
    public static Vector hadamard(Vector vector1, Vector vector2) { return new Vector(vector1.x() * vector2.x(), vector1.y() * vector2.y(), vector1.z() * vector2.z()); }
    /// Computes the dot-product of two [Vector], producing a scalar.
    ///
    /// [Wikipedia Dot Product](https://en.wikipedia.org/wiki/Dot_product)
    /// @param vector1 First vector
    /// @param vector2 Second vector
    /// @return The dot-product of the two vectors
    public static double dot(Vector vector1, Vector vector2) { return (vector1.x() * vector2.x()) + (vector1.y() * vector2.y()) + (vector1.z() * vector2.z()); }
    /// Computes the cross-product of two [Vectors](Vector), producing a [Vector].
    ///
    /// [Wikipedia Cross Product](https://en.wikipedia.org/wiki/Cross_product)
    /// @param vector1 First vector
    /// @param vector2 Second vector
    /// @return The cross-product of the two vectors
    public static Vector cross(Vector vector1, Vector vector2) { return new Vector(vector1.y() * vector2.z() - vector1.z() * vector2.y(), vector1.z() * vector2.x() - vector1.x() * vector2.z(), vector1.x() * vector2.y() - vector1.y() * vector2.x());}

    /// Reflects a [Vector] over the x-axis.
    /// @param vector The vector to reflect
    /// @return The vector reflected over the x-axis
    public static Vector reflectX(Vector vector) { return new Vector(-1 * vector.x(), vector.y(), vector.z()); }
    /// Reflects a [Vector] over the y-axis.
    /// @param vector The vector to reflect
    /// @return The vector reflected over the y-axis
    public static Vector reflectY(Vector vector) { return new Vector(vector.x(), -1 * vector.y(), vector.z()); }
    /// Reflects a [Vector] over the z-axis.
    /// @param vector The vector to reflect
    /// @return The vector reflected over the z-axis
    public static Vector reflectZ(Vector vector) { return new Vector(vector.x(), vector.y(), -1 * vector.z()); }

    /// Adds two [Matrices](Matrix).
    /// @param matrix1 First matrix
    /// @param matrix2 Second matrix
    /// @return The sum of the two matrices
    public static Matrix add(Matrix matrix1, Matrix matrix2) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix1.get(i, j) + matrix2.get(i, j);
            }
        }
        return new Matrix(temp);
    }
    /// Subtracts one [Matrix] from another.
    /// @param matrix1 First matrix
    /// @param matrix2 Second matrix
    /// @return The second matrix subtracted from the first
    public static Matrix subtract(Matrix matrix1, Matrix matrix2) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix1.get(i, j) - matrix2.get(i, j);
            }
        }
        return new Matrix(temp);
    }
    /// Scales all elements in a [Matrix] by a scalar.
    /// @param matrix A matrix
    /// @param scalar A scalar
    /// @return The matrix scaled by the scalar
    public static Matrix scale(Matrix matrix, double scalar) {
        double[][] temp = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = matrix.get(i, j) * scalar;
            }
        }
        return new Matrix(temp);
    }

    /// Computes the product of two [Matrices](Matrix).
    ///
    /// [Wikipedia Matrix Multiplication](https://en.wikipedia.org/wiki/Matrix_multiplication)
    /// @param matrix1 First matrix
    /// @param matrix2 Second matrix
    /// @return The product of the first matrix by the second
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
    /// Multiplies a [Vector] by a [Matrix], producing a [Vector].
    ///
    /// [Wikipedia Matrix Multiplication](https://en.wikipedia.org/wiki/Matrix_multiplication)
    /// @param matrix A matrix
    /// @param vector A vector
    /// @return The vector multiplied by the matrix
    public static Vector multiply(Matrix matrix, Vector vector) {
        double x = matrix.get(0, 0) * vector.x() + matrix.get(0, 1) * vector.y() + matrix.get(0, 2) * vector.z();
        double y = matrix.get(1, 0) * vector.x() + matrix.get(1, 1) * vector.y() + matrix.get(1, 2) * vector.z();
        double z = matrix.get(2, 0) * vector.x() + matrix.get(2, 1) * vector.y() + matrix.get(2, 2) * vector.z();
        return new Vector(x, y, z);
    }

    /// Rotates a [Vector] by an [Angle] theta around the z-axis.
    /// @param vector A vector
    /// @param theta An angle
    /// @return The vector rotated by theta around the z-axis
    public static Vector rotate(Vector vector, Angle theta) { return new Vector(vector.length(), add(vector.theta(), theta), vector.phi()); }

    /// Rotates a [Vector] by an [Angle] theta around the z-axis and by an
    /// [Angle] phi around the xy-plane.
    /// @param vector A vector
    /// @param theta An angle for theta
    /// @param phi An angle for phi
    /// @return The vector rotated by theta around the z-axis and phi around the xy-plane
    public static Vector rotate(Vector vector, Angle theta, Angle phi) { return new Vector(vector.length(), add(vector.theta(), theta), add(vector.phi(), phi)); }

    /// Normalizes a [Vector] to a [Vector] normal by dividing each coordinate of the [Vector] by that coordinate
    /// of the normal1 [Vector]. If the two [Vector] are equal, the returned [Vector] will be `(1, 1, 1)`.
    /// @param vector A vector to be normalized
    /// @param normal1 The vector that will be at `(1, 1, 1)` relative to the normalized vector
    /// @return The vector, normalized to the normal1 vector
    public static Vector normalize(Vector vector, Vector normal1) {
        return new Vector(vector.x() / normal1.x(), vector.y() / normal1.y(), vector.z() / normal1.z()); }

    /// Normalizes a [Vector] using a [Vector] for both normal0 and normal1. If the [Vector] to be normalized is equal to the
    /// normal0 [Vector], the result will be `(0, 0, 0)`. If the [Vector] to be normalized is equal to the normal1 [Vector],
    /// the result will be `(1, 1, 1)`.
    /// @param vector A vector to be normalized.
    /// @param normal1 The vector that will be at `(1, 1, 1)` relative to the normalized vector
    /// @param normal0 The vector that will be at `(0, 0, 0)` relative to the normalized vector
    /// @return The vector, normalized to the normal1 and normal0 vectors
    public static Vector normalize(Vector vector, Vector normal1, Vector normal0) {
        return new Vector((vector.x() - normal0.x()) / (normal1.x() - normal0.x()),
                (vector.y() - normal0.y()) / (normal1.y() - normal0.y()),
                (vector.z() - normal0.z()) / (normal1.z()) - normal0.x()); }

    /// Calculates the distance between the origin and a point at 2D coordinates `(x, y)`.
    /// @param x The x coordinate
    /// @param y The y coordinate
    /// @return The distance from the origin to a point at `(x, y)`
    public static double pythagorean(double x, double y) { return Math.sqrt((x * x) + (y * y));}

    /// Calculates the distance between the origin and a point at 3D coordinates `(x, y, z)`.
    /// @param x The x coordinate
    /// @param y The y coordinate
    /// @param z The z coordinate
    /// @return The distance from the origin to a point at `(x, y, z)`
    public static double pythagorean(double x, double y, double z) { return Math.sqrt((x*x) + (y*y) + (z*z)); }

    /// Calculates the distance between two [Vectors](Vector).
    /// @param vector1 First vector
    /// @param vector2 Second vector
    /// @return The distance between the two vectors
    public static double distance(Vector vector1, Vector vector2) { return pythagorean(vector2.x() - vector1.x(), vector2.y() - vector1.y(), vector2.z() - vector1.z()); }
}