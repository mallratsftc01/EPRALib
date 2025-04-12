package control;

import math.geometry.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

/**Extends the Gamepad Class.
 * <p></p>
 * Introduces new functionality to joysticks, triggers, and buttons.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Controller extends Gamepad {
    Gamepad gamepad;
    /**An enum to store all the buttons and analogs.*/
    public enum Key {
        A,
        B,
        X,
        Y,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        BUMPER_LEFT,
        BUMPER_RIGHT,
        STICK_LEFT,
        STICK_RIGHT,
        LEFT_STICK_X,
        RIGHT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER;

        Key() {}
    }
    /**An enum to store both joysticks.*/
    public enum Stick {
        RIGHT_STICK,
        LEFT_STICK
    }

    /**A map containing all of the buttons and corresponding keys.*/
    public Map<Key, ButtonBase> map = new HashMap<>();
    public Map<Stick, VectorButton> stick = new HashMap<>();

    private float deadband;

    /**Extends the Gamepad Class.
     * <p></p>
     * Introduces new functionality to joysticks, triggers, and buttons.
     * @param deadbandIn The starting deadband range.
     * @param g The gamepad this controller instance will extend.
     * */
    public Controller(Gamepad g, float deadbandIn) {
        gamepad = g;
        deadband = deadbandIn;
        map.put(Key.A, new BooleanButton(g, gp -> gp.a));
        map.put(Key.B, new BooleanButton(g, gp -> gp.b));
        map.put(Key.X, new BooleanButton(g, gp -> gp.x));
        map.put(Key.Y, new BooleanButton(g, gp -> gp.y));
        map.put(Key.UP, new BooleanButton(g, gp -> gp.dpad_up));
        map.put(Key.DOWN, new BooleanButton(g, gp -> gp.dpad_down));
        map.put(Key.LEFT, new BooleanButton(g, gp -> gp.dpad_left));
        map.put(Key.RIGHT, new BooleanButton(g, gp -> gp.dpad_right));
        map.put(Key.BUMPER_LEFT, new BooleanButton(g, gp -> gp.left_bumper));
        map.put(Key.BUMPER_RIGHT, new BooleanButton(g, gp -> gp.right_bumper));
        map.put(Key.STICK_LEFT, new BooleanButton(g, gp -> gp.left_stick_button));
        map.put(Key.STICK_RIGHT, new BooleanButton(g, gp -> gp.right_stick_button));
        map.put(Key.LEFT_STICK_X, new FloatButton(g, gp -> gp.left_stick_x));
        map.put(Key.RIGHT_STICK_X, new FloatButton(g, gp -> gp.right_stick_x));
        map.put(Key.LEFT_STICK_Y, new FloatButton(g, gp -> gp.left_stick_y));
        map.put(Key.RIGHT_STICK_Y, new FloatButton(g, gp -> gp.right_stick_y));
        map.put(Key.LEFT_TRIGGER, new FloatButton(g, gp -> gp.left_trigger));
        map.put(Key.RIGHT_TRIGGER, new FloatButton(g, gp -> gp.right_trigger));
        stick.put(Stick.RIGHT_STICK, new VectorButton(g, gp -> gp.right_stick_x, gp -> gp.right_stick_y));
        stick.put(Stick.LEFT_STICK, new VectorButton(g, gp -> gp.left_stick_x, gp -> gp.left_stick_y));
    }

    /**Returns the float value of an analog.
     * @param analog Corresponding key for analog.*/
    public float getAnalog(Key analog) { return map.get(analog).getFloat(); }
    /**@param joystick Corresponding stick for joystick.
     * @return The vector associated with the stick.*/
    public Vector getAnalog(Stick joystick) { return stick.get(joystick).getVector(); }
    /**Returns the boolean value of a button.
     * @param button Corresponding key for button.*/
    public boolean getButton(Key button) { return map.get(button).getBoolean(); }
    /**Returns the value of a button and an int.
     * @param button Corresponding key for button.*/
    public int getButtonInt(Key button) { return boolToInt(getButton(button)); }

    /**Sets deadband limit for joysticks and triggers.
     * @param d Deadband range.*/
    public void setDeadband(float d) { deadband = d; }
    /**Returns deadband limit for joysticks and triggers.*/
    public float getDeadband() { return deadband; }
    /**Returns 0 if in the deadband range, if not returns as normal.
     * @param analog Corresponding key for analog.*/
    public float analogDeadband(Key analog) { return (Math.abs(map.get(analog).getFloat()) > deadband) ? map.get(analog).getFloat() : 0.0F; }
    /**@param joystick Corresponding stick for joystick.
     * @return The vector associated with the stick, length is set to 0 if it was within the deadband range.*/
    public Vector analogDeadband(Stick joystick) { return (Math.abs(stick.get(joystick).getVector().getLength()) > deadband) ? stick.get(joystick).getVector() : new Vector(0.0,0.0); }
    /**Returns 0 if in the deadband range, if not returns as normal.
     * @param analog Corresponding key for analog.
     * @param deadbandIn Deadband range.*/
    public float analogDeadband(Key analog, float deadbandIn) { return (Math.abs(map.get(analog).getFloat()) > deadbandIn) ? map.get(analog).getFloat() : 0.0F; }
    /**@param joystick Corresponding stick for joystick.
     * @param deadbandIn Deadband range.
     * @return The vector associated with the stick, length is set to 0 if it was within the deadband range.*/
    public Vector analogDeadband(Stick joystick, float deadbandIn) { return (Math.abs(stick.get(joystick).getVector().getLength()) > deadbandIn) ? stick.get(joystick).getVector() : new Vector(0.0,0.0); }
    /**Returns the value raised to the power of the input.
     * @param analog Corresponding key for analog.
     * @param power Power to be raised to.*/
    public float analogPower(Key analog, float power) { return Math.signum(map.get(analog).getFloat() * (float)Math.pow(Math.abs(map.get(analog).getFloat()), power)); }
    /**@param joystick Corresponding stick for joystick
     * @param power The power to be raised to.
     * @return The vector associated with the stick, length raised to the power.*/
    public Vector analogPower(Stick joystick, float power) { return new Vector(Math.signum(stick.get(joystick).getVector().getLength() * (float)Math.pow(Math.abs(stick.get(joystick).getVector().getLength()), power)), stick.get(joystick).getVector()); }
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.
     * @param analog Corresponding key for analog.
     * @param power to be raised to.*/
    public float analogPowerDeadband(Key analog, float power) { return (Math.abs(analogPower(analog, power)) > deadband) ? analogPower(analog, power) : 0.0F; }
    /**@param joystick Corresponding stick for joystick
     * @param power The power to be raised to.
     * @return The vector associated with the stick, length raised to the power.*/
    public Vector analogPowerDeadband(Stick joystick, float power) {
        Vector v = new Vector(Math.signum(stick.get(joystick).getVector().getLength() * (float)Math.pow(Math.abs(stick.get(joystick).getVector().getLength()), power)), stick.get(joystick).getVector());
        return (Math.abs(v.getLength()) > deadband) ? v : new Vector(0,0);
    }
    /**If the value is within the deadband range, it is set to 0. If not, it is raised to the power of the input.
     * @param analog Corresponding key for analog.
     * @param power Power to be raised to.
     * @param deadbandIn Deadband range.*/
    public float analogPowerDeadband(Key analog, float power, float deadbandIn) { return (Math.abs(analogPower(analog, power)) > deadbandIn) ? analogPower(analog, power) : 0.0F; }
    /**@param joystick Corresponding stick for joystick
     * @param power The power to be raised to.
     * @param deadbandIn Deadband range.
     * @return The vector associated with the stick, length raised to the power.*/
    public Vector analogPowerDeadband(Stick joystick, float power, float deadbandIn) {
        Vector v = new Vector(Math.signum(stick.get(joystick).getVector().getLength() * (float)Math.pow(Math.abs(stick.get(joystick).getVector().getLength()), power)), stick.get(joystick).getVector());
        return (Math.abs(v.getLength()) > deadbandIn) ? v : new Vector(0,0);
    }

    /**Returns a true output only on the first call while a button is pressed.
     * If the method is called again while the button is still pressed, the return will be false.
     * If the method is called while the button is released it will clear.
     * @param button Corresponding key for button.*/
    public boolean buttonSingle(Key button) {
        return map.get(button).getSingle();
    }
    /**Returns the output of buttonSingle as an int.
     * @param button Corresponding key for button.*/
    public int buttonSingleInt(Key button) {return boolToInt(buttonSingle(button));}
    /**Will change the state of the toggle if the button is pressed.
     * Returns the new state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean buttonToggle(Key button) {
        map.get(button).toggle();
        return map.get(button).getToggle();
    }
    /**Returns the output of buttonToggle as an int.
     * @param button Corresponding key for button.*/
    public int buttonToggleInt(Key button) {return boolToInt(buttonToggle(button));}
    /**Will change the state of the toggle if the button is pressed following the rules of buttonSingle.
     * Returns the new state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean buttonToggleSingle(Key button) {
        if (buttonSingle(button)) {
            map.get(button).toggle();
        }
        return map.get(button).getToggle();
    }
    /**Returns the output of buttonToggleSingle as an int.
     * @param button Corresponding key for button.*/
    public int buttonToggleSingleInt(Key button) {return boolToInt(buttonToggleSingle(button));}
    /**Will change the state of the toggle regardless of the state of the button.
     * Returns the new state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean flipToggle(Key button) {
        map.get(button).setToggle(!(map.get(button).getToggle()));
        return map.get(button).getToggle();
    }
    /**Returns the state of the toggle without changing the state of the toggle.
     * @param button Corresponding key for button.*/
    public boolean getToggle(Key button) {
        return map.get(button).getToggle();
    }
    /**Returns the output of getToggle as an int.
     * @param button Corresponding key for button.*/
    public int getToggleInt(Key button) {return boolToInt(buttonToggleSingle(button));}
    /**If the counter is more than or equal to max it will be clear and return zero. If not, the counter will increase by one and return the result.
     * @param button Corresponding key for button.
     * @param max The maximum value of the counter.*/
    public int buttonCounter(Key button, int max) {
        if (map.get(button).getBoolean()) {
            map.get(button).tickCounter(1, max);
        }
        return map.get(button).getCounter();
    }
    /**Will perform the same action as buttonCounter but follows the rules of buttonSingle.
     * @param button Corresponding key for button.
     * @param max The maximum value of the counter.*/
    public int buttonCounterSingle(Key button, int max) {
        if (buttonSingle(button)) {
            map.get(button).tickCounter(1, max);
        }
        return map.get(button).getCounter();
    }
    /**Will increase the counter of a certain button by a certain amount. If the counter goes over max, it will clear and overflow. Returns the new value of the counter.
     * @param button Corresponding key for button.
     * @param max The maximum value of the counter.
     * @param increase The amount by which the counter will increase.*/
    public int increaseCounter(Key button, int max, int increase) {
        map.get(button).tickCounter(increase, max);
        return map.get(button).getCounter();
    }
    /**Will set the counter to a certain number.
     * @param button Corresponding key for button.
     * @param set The value to set the counter to.*/
    public void setCounter(Key button, int set) { map.get(button).setCounter(set); }
    /**Returns the current value of the counter.
     * @param button Corresponding key for button.*/
    public int getCounter(Key button) { return map.get(button).getCounter(); }

    /**If true will return 1, if false will return 0.
     * @param b The input boolean.*/
    public int boolToInt(boolean b) {return (b) ? 1 : 0;}
}