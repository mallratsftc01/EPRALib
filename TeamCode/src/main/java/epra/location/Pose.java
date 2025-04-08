package epra.location;

import epra.math.geometry.Angle;
import epra.math.geometry.Geometry;
import epra.math.geometry.Point;
import epra.math.geometry.Vector;

/**Store a pose value consisting of a point and an angle.
 * <p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Pose {

    public Point point;
    public Angle angle;

    /**Store a pose value consisting of a point and an angle.
     * @param point The point to store.
     * @param angle The angle to store.*/
    public Pose(Point point, Angle angle) {
        this.point = point;
        this.angle = angle;
    }

    /**@param point The point to store.*/
    public void setPoint(Point point) { this.point = point; }
    /**@param angle The angle to store.*/
    public void setAngle(Angle angle) { this.angle = angle; }
}
