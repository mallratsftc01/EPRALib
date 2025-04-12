package math.geometry;

/**A class that adds many geometric functions for many uses.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Geometry {
    /**A unit vector pointing to the right.*/
    public static final Vector I_HAT = new Vector(1, new Angle(0.0));
    /**A unit vector pointing upwards.*/
    public static final Vector J_HAT = new Vector(1, new Angle(90.0));
    /**Represents an angle of pi radians or 180.0 degrees.*/
    public static final Angle PI = new Angle((float)Math.PI);
    /**Represents an angle of 1.0 radian or ~57.296 degrees.*/
    public static final Angle RAD = new Angle(1.0f);

    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The resulting angle from adding the first to the second, in a range between 0 and 360 degrees (0 to 2pi).*/
    public static Angle add(Angle angle1, Angle angle2) { return new Angle((angle1.getDegree() + angle2.getDegree()) % 360); }
    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The resulting angle from subtracting the second from the first, in a range between 0 and 360 degrees (0 to 2pi).*/
    public static Angle subtract(Angle angle1, Angle angle2) { return new Angle((angle1.getDegree() - angle2.getDegree()) % 360); }
    /**@param angle Array of angles.
     * @return The average angle of the array.*/
    public static Angle average(Angle[] angle) {
        double degrees = 0.0;
        for (Angle a : angle) { degrees += a.getDegree(); }
        degrees /= angle.length;
        return new Angle(degrees % 360.0);
    }

    /**@param angle An angle.
     * @return The trigonometric sine value of an angle.*/
    public static double sin(Angle angle) { return Math.sin(angle.getRadian()); }
    /**@param angle An angle.
     * @return The trigonometric cosine value of an angle.*/
    public static double cos(Angle angle) { return Math.cos(angle.getRadian()); }
    /**@param angle An angle.
     * @return The trigonometric tangent value of an angle.*/
    public static double tan(Angle angle) { return Math.tan(angle.getRadian()); }
    /**@param angle An angle.
     * @return The trigonometric cosecant value of an angle.*/
    public static double csc(Angle angle) { return 1.0 / Math.sin(angle.getRadian()); }
    /**@param angle An angle.
     * @return The trigonometric secant value of an angle.*/
    public static double sec(Angle angle) { return 1.0 / Math.cos(angle.getRadian()); }
    /**@param angle An angle.
     * @return The trigonometric cotangent value of an angle.*/
    public static double cot(Angle angle) { return 1.0 / Math.tan(angle.getRadian()); }
    /**@param a The value whose arc sine is to be returned.
     * @return The arc sine of a value; the returned angle is in the range -pi/2 to pi/2.*/
    public static Angle asin(double a) { return new Angle((float) Math.asin(a)); }
    /**@param a The value whose arc cosine is to be returned.
     * @return The arc cosine of a value; the returned angle is in the range 0.0 to pi.*/
    public static Angle acos(double a) { return new Angle((float) Math.acos(a)); }
    /**@param a The value whose arc tangent is to be returned.
     * @return The arc tangent of a value; the returned angle is in the range -pi/2 to pi/2.*/
    public static Angle atan(double a) { return new Angle((float) Math.atan(a)); }
    /**Finds the angle of a vector from a point to (0,0). If the point is (0,0), will return an angle of 0.
     * @param p The point whose arc tangent is to be returned.
     * @return The arc tangent of a value; the returned angle is in the range 0.0 to 2pi.*/
    public static Angle atan(Point p) {
        return new Angle (
                (float) switch (p.quadrant()) {
                    case 0 -> (p.x == 0) ? (p.y == 0) ? 0 : (p.y > 0) ? Math.PI / 2 : -1 * Math.PI / 2 : (p.x > 0) ? 0 : 3 * Math.PI;
                    case 1, 4 -> Math.atan(p.y / p.x);
                    case 2, 3 -> Math.atan(p.y / p.x) + Math.PI;
                    default -> Math.atan(p.y / p.x);
            }
        );
    }

    /**@param point1 First point.
     * @param point2 Second point.
     * @return The resulting point from adding the first to the second.*/
    public static Point add(Point point1, Point point2) { return new Point(point1.x + point2.x, point1.y + point2.y); }
    /**@param point1 First point.
     * @param point2 Second point.
     * @return The resulting point from subtracting the second from the first.*/
    public static Point subtract(Point point1, Point point2) { return new Point(point1.x - point2.x, point1.y - point2.y); }
    /**@param point Array of points.
     * @return The average point of the array.*/
    public static Point average(Point[] point) {
        double x = 0.0;
        double y = 0.0;
        for (Point p : point) {
            x += p.x;
            y += p.y;
        }
        x /= point.length;
        y /= point.length;
        return new Point(x, y);
    }
    /**Flips the x and y of a point
     * @param p A point
     * @return The reversed point*/
    public static Point reverse (Point p) { return new Point(p.y, p.x); }
    /**@param point A point.
     * @param angle An angle.
       @return The point rotated by the angle about the origin.*/
    public static Point rotate(Point point, Angle angle) { return new Point(((point.x * cos(angle)) - (point.y * sin(angle))), ((point.x * sin(angle)) + (point.y * cos(angle)))); }

    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return The resulting point from adding the first to the second.*/
    public static Vector add(Vector vector1, Vector vector2) { return new Vector(add(vector1.toPoint(), vector2.toPoint())); }
    /**@param vector A vector.
     * @param angle An angle.
     * @return The resulting vector from adding the angle of the vector to the angle and keeping the length of the vector.*/
    public static Vector add(Vector vector, Angle angle) { return new Vector(vector.getLength(), add((Angle) vector, angle)); }
    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return The resulting point from subtracting the second from the first.*/
    public static Vector subtract(Vector vector1, Vector vector2) { return new Vector(subtract(vector1.toPoint(), vector2.toPoint())); }
    /**@param vector A vector.
     * @param angle An angle.
     * @return The resulting vector from subtracting the angle from the angle of the vector and keeping the length of the vector.*/
    public static Vector subtract(Vector vector, Angle angle) { return new Vector(vector.getLength(), subtract((Angle) vector, angle)); }
    /**Scales a vector by a scalar.
     * @param vector A vector.
     * @param scalar A double scalar.
     * @return The vector scaled by the scalar.*/
    public static Vector scale(Vector vector, double scalar) { return new Vector(vector.getLength() * scalar, vector); }
    /**@param vector1 First vector.
     * @param vector2 Second vector.
     * @return Returns the dot product of the two vectors.*/
    public static double dot(Vector vector1, Vector vector2) { return (vector1.toPoint().x * vector2.toPoint().x) + (vector1.toPoint().y * vector2.toPoint().y); }

    /**@param a Point a.
     * @param b Point b.
     * @return The distance between point a and b.*/
    public static double pythagorean(Point a, Point b) { return pythagorean(b.x - a.x, b.y - a.y); }
    /**@param a Length of leg a.
     * @param b Length of leg b.
     * @return The length of the hypotenuse of a triangle with legs a and b.*/
    public static double pythagorean(double a, double b) { return Math.sqrt((a * a) + (b * b));}

    /**@param angle1 First angle.
     * @param angle2 Second angle.
     * @return The direction for the first angle to reach the second angle the quickest (1.0 is clockwise, -1.0 is counterclockwise).*/
    public static double direction(Angle angle1, Angle angle2) {
        double sign = Math.signum(angle1.getDegree() - angle2.getDegree());
        if (Math.max(angle1.getDegree() % 360, angle2.getDegree() % 360) > 270 && Math.min(angle1.getDegree() % 360, angle2.getDegree() % 360) < 90) {
            sign *= -1;
        }
        return sign;
    }
}