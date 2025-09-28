package com.epra.epralib.ftclib.location;

import com.epra.epralib.ftclib.math.geometry.Vector;

import com.epra.epralib.ftclib.math.geometry.Angle;
import com.epra.epralib.ftclib.storage.PoseData;

/**Store a pose value consisting of a Vector and an angle.
 * <p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Pose {

    public Vector pos;
    public Angle angle;

    /**Store a pose value consisting of a position and an angle.
     * @param pos The position to store.
     * @param angle The angle to store.*/
    public Pose(Vector pos, Angle angle) {
        this.pos = pos;
        this.angle = angle;
    }

    /**@return This pose as a PoseData record.*/
    public PoseData toPoseData() {
        return new PoseData(pos.x(), pos.y(), angle.degree());
    }
}
