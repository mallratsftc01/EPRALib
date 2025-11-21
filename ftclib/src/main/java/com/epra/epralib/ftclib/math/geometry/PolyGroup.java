package com.epra.epralib.ftclib.math.geometry;

import java.util.ArrayList;
import java.util.Collections;

/**Holds a group of geometric components.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class PolyGroup implements Shape2D {

    /**All the geometric components of the PolyGroup.*/
    ArrayList<Shape2D> components = new ArrayList<>();

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
        for (Shape2D component : components) {
            area += component.getArea();
        }
        return area;
    }

    /**@param point Point to check.
     * @return True if the Vector is within the PolyGroup, false if not.*/
    public boolean checkPoint(Vector point) {
        for (Shape2D component : components) {
            if (component.checkPoint(point)) {
                return true;
            }
        }
        return false;
    }
}