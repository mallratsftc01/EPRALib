package math.geometry;

import java.util.ArrayList;
import java.util.Collections;

/**Holds a group of geometric components.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class PolyGroup implements Shape2D {

    /**All the geometric components of the PolyGroup.*/
    ArrayList<Shape2D> components = new ArrayList<Shape2D>();

    /**Holds a group of geometric components.
     *@param components Geometric components to form the PolyGroup.*/
    public PolyGroup(Shape2D[] components) { Collections.addAll(this.components, components); }
    /**Holds a group of geometric components.*/
    public PolyGroup() {}

    /**@param component Geometric component to add to the PolyGroup.*/
    public void addComponent(Shape2D component) { components.add(component); }
    /**@param components Array of geometric components to add to the PolyGroup.*/
    public void addComponent(Shape2D[] components) { Collections.addAll(this.components, components); }
    /**Clears all components from the PolyGroup.*/
    public void clear() { components.clear(); }
    /**@return The ArrayList holding the components of the PolyGroup.*/
    public ArrayList<Shape2D> getComponents() { return components; }


    /**@return The area of the PolyGroup. Overlapped area between components may be over counted, leading to inaccurate counts. To be fixed.*/
    public double getArea() {
        double area = 0.0;
        for (int i = 0; i < components.size(); i++) { area += components.get(i).getArea(); }
        return area;
    }

    /**@param point Point to check.
     * @return True if the point is within the PolyGroup, false if not.*/
    public boolean checkPoint(Point point) {
        for (int i = 0; i < components.size(); i++) { if (components.get(i).checkPoint(point)) { return true; } }
        return false;
    }
}