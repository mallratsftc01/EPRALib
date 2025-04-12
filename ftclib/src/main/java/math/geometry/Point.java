package math.geometry;

/**Stores an x,y coordinate point.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Point {

    public double x,y;

    /**Stores an x,y coordinate point.
     * @param x
     * @param y*/
    public Point (double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**Tests if a point is the same as this point.\
     * @param p Point to test
     * @return True if the points are the same, false if they are not.*/
    public boolean equals(Point p) { return (p.x == x && p.y == y); }

    /**Returns the quadrant of the point. If either the x or y of the point is 0, instead returns 0.
     * <p>
     * 2 | 1
     * <p>
     * - 0 -
     * </p>
     * 3 | 4
     * @return The quadrant this point is in.*/
    public int quadrant() {
        if (x > 0) {
            if (y > 0) { return 1; }
            else if (y < 0) { return 4; }
        } else if (x < 0) {
            if (y > 0) { return 2; }
            else if (y < 0) { return 3; }
        }
        return 0;
    }
}