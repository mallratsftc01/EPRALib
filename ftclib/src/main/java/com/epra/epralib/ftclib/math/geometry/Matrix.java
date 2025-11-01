package com.epra.epralib.ftclib.math.geometry;

/**Stores a three-dimensional matrix.
 * <p></p>
 *  *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Matrix {

    /**The identity matrix. */
    public static final Matrix IDENTITY = new Matrix(new double[][]{{1,0,0},{0,1,0},{0,0,1}});

    /**Rotates the identity matrix around the z-axis by the angle.
     * @param angle The angle to rotate around the z-axis.
     * @return A new matrix that represents a rotation by the angle around the z-axis*/
    public static Matrix yaw(Angle angle) {
        return new Matrix(new double[][]{
                {Math.cos(angle.radian()), -Math.sin(angle.radian()), 0},
                {Math.sin(angle.radian()), Math.cos(angle.radian()), 0},
                {0, 0, 1}
        });
    }
    /**Rotates the identity matrix around the x-axis by the angle.
     * @param angle The angle to rotate around the x-axis.
     * @return A new matrix that represents a rotation by the angle around the x-axis*/
    public static Matrix pitch(Angle angle) {
        return new Matrix(new double[][]{
                {1, 0, 0},
                {0, Math.cos(angle.radian()), -Math.sin(angle.radian())},
                {0, Math.sin(angle.radian()), Math.cos(angle.radian())}
        });
    }
    /**Rotates the identity matrix around the y-axis by the angle.
     * @param angle The angle to rotate around the y-axis.
     * @return A new matrix that represents a rotation by the angle around the y-axis*/
    public static Matrix roll(Angle angle) {
        return new Matrix(new double[][]{
                {Math.cos(angle.radian()), 0, Math.sin(angle.radian())},
                {0, 1, 0},
                {-Math.sin(angle.radian()), 0, Math.cos(angle.radian())}
        });
    }

    /**Returns a matrix that will return the vector (1, 1, 1) when multiplied with the provided vector.
     * @param vector A vector.
     * @return The inverse matrix of the vector.*/
    public static Matrix normal(Vector vector) {
        return new Matrix(new double[][]{
                {1 / vector.x(), 0, 0},
                {0, 1 / vector.y(), 0},
                {0, 0, 1 / vector.z()}
        });
    }

    private double[][] values;

    /**Stores a matrix.
     * @param values The values that will fill the matrix. Only the first three rows and first three columns of those rows will be used. Empty values will be set to 0.*/
    public Matrix(double[][] values) {
        this.values = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (i < values.length && j < values[i].length) {
                    this.values[i][j] = values[i][j];
                } else {
                    this.values[i][j] = 0.0;
                }
            }
        }
    }

    /**Gets the value of the matrix at the row and column.
     * @param row The row of the matrix. Must be less than 3.
     * @param col The column of the matrix. Must be less than 3.
     * @return The value of the matrix at row, col.*/
    public double get(int row, int col) {
        return values[row][col];
    }

    /**Returns the matrix as a 3 by 3 array.
     * @return The matrix as an array.*/
    public double[][] get() {
        return values;
    }
}
