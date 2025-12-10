package com.epra.epralib.ftclib.control.gamepad_elements;

import com.epra.epralib.ftclib.control.Controller;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

/// Represents a button on a [Gamepad].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see Controller
public class Button implements GamepadElement {

    Supplier<Boolean> f;

    private boolean flag;
    private boolean toggle;
    private int counter;

    /// Represents a button on a [Gamepad].
    /// @param f A boolean supplier for the current state of the button
    /// @see Controller
    public Button(Supplier<Boolean> f) {
        this.f = f;
        flag = false;
        toggle = false;
        counter = 0;
    }

    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public boolean getBoolean() { return f.get(); }
    /// Returns 1.0 if the value of this element is `True`, 0.0 otherwise.
    /// @return {@inheritDoc}
    @Override
    public float getFloat() {
        return (f.get()) ? 1.0f : 0.0f;
    }

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