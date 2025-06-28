package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.Point;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.math.geometry.Point;
import com.epra.epralib.ftclib.storage.PoseData;

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

    /**@return This pose as a PoseData record.*/
    public PoseData toPoseData() {
        return new PoseData(point.x, point.y, angle.getDegree());
    }
}
