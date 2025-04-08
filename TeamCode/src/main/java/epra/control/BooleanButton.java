package epra.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

/**Represents a single element on the gamepad that returns a boolean (ie. dpad, bumpers).
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class BooleanButton implements ButtonBase {

    Gamepad g;
    Function<Gamepad, Boolean> f;

    private boolean flag;
    private boolean toggle;
    private int counter;

    /**Represents a single element on the gamepad that returns a boolean (ie. dpad, bumpers).
     * @param g The gamepad where this element is located.
     * @param f The function of the gamepad object that returns the value of this element.*/
    public BooleanButton(Gamepad g, Function<Gamepad, Boolean> f) {
        this.g = g;
        this.f = f;
        flag = false;
        toggle = false;
        counter = 0;
    }

    /**Returns the value of this element.
     * @return The value of this element as a boolean.*/
    @Override
    public boolean getBoolean() {
        return f.apply(g);
    }
    /**Returns 1.0 if the value of the element is true, 0.0 if not.
     * @return The value of this element as a float.*/
    @Override
    public float getFloat() {
        return (f.apply(g)) ? 1.0f : 0.0f;
    }

    /**Returns a true output only on the first call while this element has a boolean value of true.
     * If the method is called again while this element still has a boolean value of true, the return will be false.
     * If the method is called while this element has a boolean value of false it will clear.
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

    /**@return The counter value of this element.*/
    @Override
    public int getCounter() {
        return counter;
    }
    /**Sets the counter value of this element to a specified integer.
     * @param c The new value of the the counter.*/
    @Override
    public void setCounter(int c) {
        counter = c;
    }
    /**Increases the counter value of this element by the specified amount.
     * If the new value of the counter is more than or equal to the specified max the counter will roll back to 0.
     * @param i The amount to increase the counter value by.
     * @param max The maximum value of of the counter.*/
    @Override
    public void tickCounter(int i, int max) {
        counter += i;
        counter %= max;
    }
}