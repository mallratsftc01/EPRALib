package com.epra.epralib.ftclib.control.buttons;

import java.util.function.Supplier;

import com.epra.epralib.ftclib.math.geometry.Vector;

/**Represents a single element on the gamepad that returns two floats in an (x,y) pair (joysticks).
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class VectorButton implements ButtonBase {

    Supplier<Float> x;
    Supplier<Float> y;

    private boolean flag;
    private boolean toggle;
    private int counter;

    /**Represents a single element on the gamepad that returns two floats in an (x,y) pair (joysticks).
     * @param x A supplier that returns the x value of this element.
     * @param y A supplier that returns the y value of this element.*/
    public VectorButton(Supplier<Float> x, Supplier<Float> y) {
        this.x = x;
        this.y = y;
        flag = false;
        toggle = false;
        counter = 0;
    }

    /**Returns the value of this element.
     * @return The value of this element as a Vector.*/
    public Vector getVector() { return new Vector(x.get(), y.get()); }
    /**Returns true if the element's Vector's length is not zero.
     * @return The value of this element as a boolean.*/
    @Override
    public boolean getBoolean() {
        return getVector().length() != 0;
    }
    /**Returns the length of this element's Vector.
     * @return The value of this element as a float.*/
    @Override
    public float getFloat() {
        return (float) getVector().length();
    }

    /**Returns a true output only on the first call while this element has a boolean value of true.
     * If the method is called again while this element still has a boolean value of true, the return will be false.
     * If the method is called while this element has a boolean value of false, it will clear.
     * @return The single press value of this element.*/
    @Override
    public boolean getSingle() {
        if (getBoolean()) {
            if (!flag) {
                flag = true;
                return true;
            }
        } else { flag = false; }
        return false;
    }

    /**@return The toggle value of this element.*/
    @Override
    public boolean getToggle() {
        return toggle;
    }
    /**Sets the toggle value of this element to the specified boolean.
     * @param t The new toggle value.*/
    @Override
    public void setToggle(boolean t) { toggle = t; }
    /**Flips the toggle value of this element if the boolean value of this element is true.*/
    @Override
    public void toggle() {
        if (getBoolean()) { toggle = !toggle; }
    }

    /**@return The counter-value of this element.*/
    @Override
    public int getCounter() {
        return counter;
    }
    /**Sets the counter-value of this element to a specified integer.
     * @param c The new value of the counter.*/
    @Override
    public void setCounter(int c) {
        counter = c;
    }
    /**Increases the counter-value of this element by the specified amount.
     * If the new value of the counter is more than or equal to the specified max, the counter will roll back to 0.
     * @param i The amount to increase the counter-value by.
     * @param max The maximum value of the counter.*/
    @Override
    public void tickCounter(int i, int max) {
        counter += i;
        counter %= max;
    }
}