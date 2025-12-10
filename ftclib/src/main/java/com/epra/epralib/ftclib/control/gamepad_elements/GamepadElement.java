package com.epra.epralib.ftclib.control.gamepad_elements;

import com.epra.epralib.ftclib.control.Controller;
import com.qualcomm.robotcore.hardware.Gamepad;

/// An interface for representing a button or element on a [Gamepad].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see Controller
public interface GamepadElement {

    /// Returns the current value of this element as a boolean.
    /// @return The boolean value of this element
    boolean getBoolean();
    /// Returns the current value of this element as a float.
    /// @return The float value of this element
    float getFloat();

    /// Returns `True` only the first time this is called while this element's value is `True`.
    ///
    /// Calling this while this element's value is `False` will reset this method.
    /// This is useful for operations that should only happen once after a button is pressed,
    /// not everytime the code loops while the button is pressed.
    /// @return The single press value of this element
    boolean getSingle();

    /// Returns the current toggle-value of this element.
    /// @return The toggle-value of this element
    boolean getToggle();
    /// Sets the toggle-value for this element.
    /// @param t The toggle-value for this element
    void setToggle(boolean t);
    /// Will change the state of this element's toggle-value from `True` to `False` or vice versa
    /// if the value of this element is `True`.
    void toggle();

    /// Returns the current counter-value of this element.
    /// @return The counter-value of this element
    int getCounter();
    /// Sets the counter-value for this element.
    /// @param c The counter-value for this element
    void setCounter(int c);
    /// Increases the counter-value for this element.
    ///
    /// If after incrementation, the counter is more than or equal to the maximum value,
    /// it will be taken modulo `max`.
    /// @param i The amount to increase the counter-value by
    /// @param max The exclusive maximum value of the counter-value
    void tickCounter(int i, int max);
}
