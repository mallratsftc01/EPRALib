package epra.location;

import epra.math.geometry.Point;
import epra.math.geometry.Shape2D;

import java.util.HashMap;
import java.util.Map;

/**A basic interface to build field maps off of.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public interface FieldMap {

    /**Holds all the different sections of the field.*/
    enum FieldSection implements Section{
        DOES_NOT_EXIST
    }

    /**Map of all sections and associated component.*/
    Map<FieldSection, Shape2D> map = new HashMap<>();
    Shape2D obstacle = null;

    /**@param point Point to check.
     * @return Which section of the field the point falls into. Returns DOES_NOT_EXIST if the point doesn't fall into any section of the field.*/
    default Section checkPoint(Point point) {
        for (Map.Entry<FieldSection, Shape2D> entry : map.entrySet()) {
            if (entry.getValue().checkPoint(point)) { return entry.getKey(); }
        }
        return FieldSection.DOES_NOT_EXIST;
    }

    /**@param point Point to check.
     * @return True if the point is hitting an obstacle, false if not.*/
    default boolean checkObstacle(Point point) {
        return obstacle.checkPoint(point);
    }
}
