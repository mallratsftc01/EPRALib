package com.epra.epralib.ftclib.control.gamepad_elements;

import java.util.function.Supplier;

import com.epra.epralib.ftclib.control.Controller;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

/// Represents a joystick on the [Gamepad].
///
/// The position of the joystick is represented as a [Vector],
/// with the origin being the resting position of the joystick.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
/// @see Controller
public class Joystick implements GamepadElement {

    Supplier<Float> x;
    Supplier<Float> y;

    private boolean flag;
    private boolean toggle;
    private int counter;

    /// Represents a joystick on the [Gamepad].
    ///
    /// The position of the joystick is represented as a [Vector],
    /// with the origin being the resting position of the joystick.
    /// @param x A supplier for the x position of the joystick as a float
    /// @param y A supplier for the y position of the joystick as a float
    /// @see Controller
    public Joystick(Supplier<Float> x, Supplier<Float> y) {
        this.x = x;
        this.y = y;
        flag = false;
        toggle = false;
        counter = 0;
    }

    /// Returns the value of this element as a [Vector],
    /// with the origin at the resting position of the joystick.
    /// @return The vector value of this element
    public Vector getVector() { return new Vector(x.get(), y.get()); }
    /// Returns if the joystick is not at the resting position.
    /// @return {@inheritDoc}
    @Override
    public boolean getBoolean() {
        return getVector().length() != 0;
    }
    /// {@inheritDoc}
    /// @return {@inheritDoc}
    @Override
    public float getFloat() {
        return (float) getVector().length();
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