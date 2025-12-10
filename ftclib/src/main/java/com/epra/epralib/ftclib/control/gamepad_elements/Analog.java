package com.epra.epralib.ftclib.control.gamepad_elements;

import com.epra.epralib.ftclib.control.Controller;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

/// Represents an analog on a [Gamepad].
///
/// An analog is a single direction of a joystick or a trigger.
/// This type of gamepad element has a float value, as opposed to a boolean value like a [Button].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see Controller
public class Analog implements GamepadElement {

    Supplier<Float> f;

    private boolean flag;
    private boolean toggle;
    private int counter;

    /// Represents an analog on a [Gamepad].
    ///
    /// An analog is a single direction of a joystick or a trigger.
    /// This type of gamepad element has a float value, as opposed to a boolean value like a [Button].
    /// @param f A float supplier for the current state of the analog
    /// @see Controller
    public Analog(Supplier<Float> f) {
        this.f = f;
        flag = false;
        toggle = false;
        counter = 0;
    }

    /// Returns if the current value of this element is not 0.
    /// @return {@inheritDoc}
    @Override
    public boolean getBoolean() { return f.get() != 0; }
    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public float getFloat() { return f.get(); }

    /// {@inheritDoc}
    /// @return {@inheritDoc}
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

    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public boolean getToggle() {
        return toggle;
    }
    /// {@inheritDoc}
    /// @param t {@inheritDoc}
    @Override
    public void setToggle(boolean t) { toggle = t; }
    /// {@inheritDoc}
    @Override
    public void toggle() {
        if (getBoolean()) { toggle = !toggle; }
    }

    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public int getCounter() {
        return counter;
    }
    /// {@inheritDoc}
    /// @param c {@inheritDoc}
    @Override
    public void setCounter(int c) {
        counter = c;
    }
    /// {@inheritDoc}
    /// @param i {@inheritDoc}
    /// @param max {@inheritDoc}
    @Override
    public void tickCounter(int i, int max) {
        counter += i;
        counter %= max;
    }
}