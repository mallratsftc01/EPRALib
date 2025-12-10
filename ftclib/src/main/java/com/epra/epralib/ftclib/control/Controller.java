package com.epra.epralib.ftclib.control;

import com.epra.epralib.ftclib.control.gamepad_elements.Button;
import com.epra.epralib.ftclib.control.gamepad_elements.GamepadElement;
import com.epra.epralib.ftclib.control.gamepad_elements.Analog;
import com.epra.epralib.ftclib.control.gamepad_elements.Joystick;
import com.epra.epralib.ftclib.math.geometry.Vector;
import com.epra.epralib.ftclib.storage.logdata.DataLogger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

/// Extends and gives new functionality to the [Gamepad] class.
/// 
/// Includes several new ways to use buttons, joysticks, and triggers.
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class Controller extends Gamepad implements DataLogger {
    Gamepad gamepad;
    /// All the data types that can be taken directly from the [Gamepad]'s keys.
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
    /// Joysticks as one object, as opposed to seperated X and Y.
    public enum Stick {
        RIGHT_STICK,
        LEFT_STICK
    }

    private final Map<Key, GamepadElement> keys = new HashMap<>();
    private final Map<String, GamepadElement> chords = new HashMap<>();
    private final Map<Stick, Joystick> sticks = new HashMap<>();

    private final float deadband;

    private final String logPath;
    private final Key[] loggingTargets;
    private final HashMap<String, Double> logData;

    /// Extends and gives new functionality to the [Gamepad] class.
    ///
    /// Includes several new ways to use buttons, joysticks, and triggers.
    /// @param deadbandIn The starting deadband range
    /// @param gamepad The gamepad this controller instance will extend
    /// @param id A string that identifies the log files of this Controller
    /// @param loggingTargets An array of all [Key]s whose outputs should be logged
    public Controller(Gamepad gamepad, float deadbandIn, String id, Key[] loggingTargets) {
        this.gamepad = gamepad;
        deadband = deadbandIn;
        keys.put(Key.A, new Button(() -> gamepad.a));
        keys.put(Key.B, new Button(() -> gamepad.b));
        keys.put(Key.X, new Button(() -> gamepad.x));
        keys.put(Key.Y, new Button(() -> gamepad.y));
        keys.put(Key.UP, new Button(() -> gamepad.dpad_up));
        keys.put(Key.DOWN, new Button(() -> gamepad.dpad_down));
        keys.put(Key.LEFT, new Button(() -> gamepad.dpad_left));
        keys.put(Key.RIGHT, new Button(() -> gamepad.dpad_right));
        keys.put(Key.BUMPER_LEFT, new Button(() -> gamepad.left_bumper));
        keys.put(Key.BUMPER_RIGHT, new Button(() -> gamepad.right_bumper));
        keys.put(Key.STICK_LEFT, new Button(() -> gamepad.left_stick_button));
        keys.put(Key.STICK_RIGHT, new Button(() -> gamepad.right_stick_button));
        keys.put(Key.LEFT_STICK_X, new Analog(() -> gamepad.left_stick_x));
        keys.put(Key.RIGHT_STICK_X, new Analog(() -> gamepad.right_stick_x));
        keys.put(Key.LEFT_STICK_Y, new Analog(() -> gamepad.left_stick_y));
        keys.put(Key.RIGHT_STICK_Y, new Analog(() -> gamepad.right_stick_y));
        keys.put(Key.LEFT_TRIGGER, new Analog(() -> gamepad.left_trigger));
        keys.put(Key.RIGHT_TRIGGER, new Analog(() -> gamepad.right_trigger));
        sticks.put(Stick.RIGHT_STICK, new Joystick(() -> gamepad.right_stick_x, () -> gamepad.right_stick_y));
        sticks.put(Stick.LEFT_STICK, new Joystick(() -> gamepad.left_stick_x, () -> gamepad.left_stick_y));

        this.logPath = "Controller_" + id;
        this.loggingTargets = loggingTargets;
        logData = new HashMap<>();
    }
    /// Extends and gives new functionality to the [Gamepad] class.
    ///
    /// Includes several new ways to use buttons, joysticks, and triggers.
    /// 
    /// Will log the outputs of no [Key]s.
    /// @param deadbandIn The starting deadband range
    /// @param gamepad The gamepad this controller instance will extend
    /// @param id A string that identifies the log files of this Controller
    public Controller(Gamepad gamepad, float deadbandIn, String id) {
        this(gamepad, deadbandIn, id, new Key[0]);
    }

    /// Returns the float value of an [Analog].
    /// @param analog Corresponding key for analog
    /// @return The float value of the analog
    public float getAnalog(Key analog) { return keys.get(analog).getFloat(); }
    /// Returns the [Vector] value of a [Joystick].
    /// @param joystick Corresponding stick for joystick
    /// @return The vector associated with the stick
    public Vector getAnalog(Stick joystick) { return sticks.get(joystick).getVector(); }

    /// Returns deadband limit for [Analog]s.
    /// @return The deadband range
    public float getDeadband() { return deadband; }
    /// Returns 0 if in the deadband range, otherwise returns the same as [#getAnalog(Key)].
    /// @param analog Corresponding key for analog
    /// @return The deadbanded float value of the analog
    public float analogDeadband(Key analog) { return (Math.abs(keys.get(analog).getFloat()) > deadband) ? keys.get(analog).getFloat() : 0.0F; }
    /// Returns (0,0) if in the deadband range, otherwise returns the same as [#getAnalog(Stick)].
    /// @param joystick Corresponding stick for joystick
    /// @return The vector associated with the stick, length is set to 0 if it was within the deadband range
    public Vector analogDeadband(Stick joystick) { return (Math.abs(sticks.get(joystick).getVector().length()) > deadband) ? sticks.get(joystick).getVector() : new Vector(0.0,0.0); }
    /// Returns 0 if in the deadband range, otherwise returns the same as [#getAnalog(Key)].
    /// @param analog Corresponding key for analog
    /// @param deadbandIn Deadband range
    /// @return The deadbanded float value of the analog
    public float analogDeadband(Key analog, float deadbandIn) { return (Math.abs(keys.get(analog).getFloat()) > deadbandIn) ? keys.get(analog).getFloat() : 0.0F; }
    /// Returns (0,0) if in the deadband range, otherwise returns the same as [#getAnalog(Stick)].
    /// @param joystick Corresponding stick for joystick
    /// @param deadbandIn Deadband range
    /// @return The vector associated with the stick, length is set to 0 if it was within the deadband range
    public Vector analogDeadband(Stick joystick, float deadbandIn) { return (Math.abs(sticks.get(joystick).getVector().length()) > deadbandIn) ? sticks.get(joystick).getVector() : new Vector(0.0,0.0); }
    /// Returns the value of [#getAnalog(Key)] raised to a certain power.
    /// @param analog Corresponding key for analog
    /// @param power Power to be raised to
    /// @return The float value of the analog, raised to the power
    public float analogPower(Key analog, float power) { return Math.signum(keys.get(analog).getFloat() * (float)Math.pow(Math.abs(keys.get(analog).getFloat()), power)); }
    /// Returns the value of [#getAnalog(Stick)], with the length raised to a certain power.
    /// @param joystick Corresponding stick for joystick
    /// @param power Power to be raised to
    /// @return The float value of the analog, raised to the power
    public Vector analogPower(Stick joystick, float power) { return new Vector(Math.pow(sticks.get(joystick).getVector().length(), power), sticks.get(joystick).getVector().theta()); }
    /// Returns the value of [#getAnalog(Key)] raised to a certain power, with a deadband applied.
    /// @param analog Corresponding key for analog
    /// @param power Power to be raised to
    /// @return The float value of the analog, raised to the power
    ///
    /// @see #analogDeadband(Key)
    public float analogPowerDeadband(Key analog, float power) { return (Math.abs(analogPower(analog, power)) > deadband) ? analogPower(analog, power) : 0.0F; }
    /// Returns the value of [#getAnalog(Stick)], with the length raised to a certain power, with a deadband applied.
    /// @param joystick Corresponding stick for joystick
    /// @param power Power to be raised to
    /// @return The float value of the analog, raised to the power
    ///
    /// @see #analogDeadband(Stick)
    public Vector analogPowerDeadband(Stick joystick, float power) { return analogPowerDeadband(joystick, power, deadband); }
    /// Returns the value of [#getAnalog(Key)] raised to a certain power, with a deadband applied.
    /// @param analog Corresponding key for analog
    /// @param power Power to be raised to
    /// @param deadbandIn Deadband range
    /// @return The float value of the analog, raised to the power
    ///
    /// @see #analogDeadband(Key, float)
    public float analogPowerDeadband(Key analog, float power, float deadbandIn) { return (Math.abs(analogPower(analog, power)) > deadbandIn) ? analogPower(analog, power) : 0.0F; }
    /// Returns the value of [#getAnalog(Stick)], with the length raised to a certain power, with a deadband applied.
    /// @param joystick Corresponding stick for joystick
    /// @param power Power to be raised to
    /// @param deadbandIn Deadband range
    /// @return The float value of the analog, raised to the power
    ///
    /// @see #analogDeadband(Stick)
    public Vector analogPowerDeadband(Stick joystick, float power, float deadbandIn) {
        Vector v = analogPower(joystick, power);
        return (v.length() > deadbandIn) ? v : new Vector(0,0);
    }

    /// Returns the boolean value of a [Button].
    /// @param button Corresponding key for the button
    /// @return If the button is currently pressed
    public boolean getButton(Key button) { return keys.get(button).getBoolean(); }

    /// Returns the boolean value of a chord.
    /// @param chord Corresponding id for the chord
    /// @return If the chord is currently pressed
    public boolean getButton(String chord) { return chords.get(chord).getBoolean(); }

    /// Returns `True` only the first time this method is called while the [Button] is pressed.
    /// @param button Corresponding key for the button
    /// @return The single press value of the button
    public boolean buttonSingle(Key button) {
        return keys.get(button).getSingle();
    }

    /// Returns `True` only the first time this method is called while the chord is pressed.
    /// @param chord Corresponding id for the chord
    /// @return The single press value of the chord
    public boolean buttonSingle(String chord) {
        return chords.get(chord).getSingle();
    }

    /// Will change the state of the toggle from `True` to `False` or vice versa when the [Button] is pressed.
    /// @param button Corresponding key for the button
    /// @return The current value of the toggle
    public boolean buttonToggle(Key button) {
        keys.get(button).toggle();
        return keys.get(button).getToggle();
    }
    /// Will change the state of the toggle from `True` to `False` or vice versa when the chord is pressed.
    /// @param chord Corresponding id for the chord
    /// @return The current value of the chord
    public boolean buttonToggle(String chord) {
        chords.get(chord).toggle();
        return chords.get(chord).getToggle();
    }
    /// Will change the state of the toggle from `True` to `False` or vice versa when the [Button] is pressed
    /// following the rules of [#buttonSingle(Key)].
    /// @param button Corresponding key for the button
    /// @return The current value of the toggle
    public boolean buttonToggleSingle(Key button) {
        if (buttonSingle(button)) {
            keys.get(button).toggle();
        }
        return keys.get(button).getToggle();
    }
    /// Will change the state of the toggle from `True` to `False` or vice versa when the chord is pressed
    /// following the rules of [#buttonSingle(String)].
    /// @param chord Corresponding id for the chord
    /// @return The current value of the toggle
    public boolean buttonToggleSingle(String chord) {
        if (buttonSingle(chord)) {
            chords.get(chord).toggle();
        }
        return chords.get(chord).getToggle();
    }
    /// Will change the state of the toggle from `True` to `False` or vice versa.
    /// @param button Corresponding key for the button
    /// @return The current value of the toggle
    public boolean flipToggle(Key button) {
        keys.get(button).setToggle(!(keys.get(button).getToggle()));
        return keys.get(button).getToggle();
    }
    /// Will change the state of the toggle from `True` to `False` or vice versa.
    /// @param chord Corresponding id for the chord
    /// @return The current value of the toggle
    public boolean flipToggle(String chord) {
        chords.get(chord).setToggle(!(chords.get(chord).getToggle()));
        return chords.get(chord).getToggle();
    }
    /// Returns the state of the toggle without changing it.
    /// @param button Corresponding key for the button
    /// @return The current value of the toggle
    public boolean getToggle(Key button) {
        return keys.get(button).getToggle();
    }
    /// Returns the state of the toggle without changing it.
    /// @param chord Corresponding id for the chord
    /// @return The current value of the toggle
    public boolean getToggle(String chord) {
        return chords.get(chord).getToggle();
    }
    /// Modifies an integer counter attached to the [Button].
    ///
    /// If the button is pressed when this is called, the counter will be increased by 1.
    /// If after incrementation, the counter is more than or equal to the maximum value, it will be reset back to 0.
    /// @param button Corresponding key for the button
    /// @param max The maximum value of the counter
    /// @return The current value of the counter
    public int buttonCounter(Key button, int max) {
        if (keys.get(button).getBoolean()) {
            keys.get(button).tickCounter(1, max);
        }
        return keys.get(button).getCounter();
    }
    /// Modifies an integer counter attached to the chord.
    ///
    /// If the chord is pressed when this is called, the counter will be increased by 1.
    /// If after incrementation, the counter is more than or equal to the maximum value, it will be reset back to 0.
    /// @param chord Corresponding id for the chord
    /// @param max The maximum value of the counter
    /// @return The current value of the counter
    public int buttonCounter(String chord, int max) {
        if (chords.get(chord).getBoolean()) {
            chords.get(chord).tickCounter(1, max);
        }
        return chords.get(chord).getCounter();
    }
    /// Modifies an integer counter attached to the [Button].
    ///
    /// If [#buttonSingle(Key)] is `True` for this button when this is called, the counter will be increased by 1.
    /// If after incrementation, the counter is more than or equal to the maximum value, it will be reset back to 0.
    /// @param button Corresponding key for the button
    /// @param max The maximum value of the counter
    /// @return The current value of the counter
    public int buttonCounterSingle(Key button, int max) {
        if (buttonSingle(button)) {
            keys.get(button).tickCounter(1, max);
        }
        return keys.get(button).getCounter();
    }
    /// Modifies an integer counter attached to the chord.
    ///
    /// If [#buttonSingle(String)] is `True` for this chord when this is called, the counter will be increased by 1.
    /// If after incrementation, the counter is more than or equal to the maximum value, it will be reset back to 0.
    /// @param chord Corresponding id for the chord
    /// @param max The maximum value of the counter
    /// @return The current value of the counter
    public int buttonCounterSingle(String chord, int max) {
        if (buttonSingle(chord)) {
            chords.get(chord).tickCounter(1, max);
        }
        return chords.get(chord).getCounter();
    }
    /// Modifies an integer counter attached to the [Button].
    ///
    /// Increases the counter by the specified amount.
    /// If after incrementation, the counter is more than or equal to the maximum value, it will be reset back to 0.
    /// @param button Corresponding key for the button
    /// @param max The maximum value of the counter
    /// @param increase The amount by which the counter will increase
    /// @return The current value of the counter
    public int increaseCounter(Key button, int max, int increase) {
        keys.get(button).tickCounter(increase, max);
        return keys.get(button).getCounter();
    }
    /// Modifies an integer counter attached to the chord.
    ///
    /// Increases the counter by the specified amount.
    /// If after incrementation, the counter is more than or equal to the maximum value, it will be reset back to 0.
    /// @param chord Corresponding id for the chord
    /// @param max The maximum value of the counter
    /// @param increase The amount by which the counter will increase
    /// @return The current value of the counter
    public int increaseCounter(String chord, int max, int increase) {
        chords.get(chord).tickCounter(increase, max);
        return chords.get(chord).getCounter();
    }
    /// Modifies an integer counter attached to the [Button].
    ///
    /// Sets the counter to a specified value.
    /// @param button Corresponding key for the button
    /// @param set The number to set the counter to
    public void setCounter(Key button, int set) { keys.get(button).setCounter(set); }
    /// Modifies an integer counter attached to the chord.
    ///
    /// Sets the counter to a specified value.
    /// @param chord Corresponding id for the chord
    /// @param set The number to set the counter to
    public void setCounter(String chord, int set) { chords.get(chord).setCounter(set); }
    ///Returns the current value of the counter attached to the [Button].
    /// @param button Corresponding key for the button
    /// @return The current value of the counter
    public int getCounter(Key button) { return keys.get(button).getCounter(); }
    ///Returns the current value of the counter attached to the chord.
    /// @param chord Corresponding id for the chord
    /// @return The current value of the counter
    public int getCounter(String chord) { return chords.get(chord).getCounter(); }

    /// Creates a new chord of buttons that acts as a single [Button].
    ///
    /// A chord acts as a button only considered pressed if all buttons
    /// that compose it are pressed at the same time.
    ///
    /// Will fail if less than two keys are provided or the given `id` is already used by a different chord.
    /// @param id A tag that will identify this chord
    /// @param keys Two or more keys to make up this chord
    /// @return True if the chord was successfully created
    public boolean createChord(String id, Key... keys) {
        if (keys.length < 2 || chords.containsKey(id)) { return false; }
        chords.put(id, new Button(() -> {
            boolean b = true;
            for (Key k : keys) {
                b = b && this.keys.get(k).getBoolean();
            }
            return b;
        }));
        return true;
    }

    /// {@inheritDoc}
    /// @return The relative file path for the logs from this logger
    @Override
    public String getLogPath() { return logPath; }
    /// {@inheritDoc}
    /// @return `True`
    @Override
    public boolean updateLog() {
        for (Key key : loggingTargets) {
            logData.put(key.toString().toLowerCase(), (double) keys.get(key).getFloat());
        }
        return true;
    }
    /// {@inheritDoc}
    /// @return A hash map with a data snapshot of this logger
    @Override
    public HashMap<String, Double> logData(){
        return logData;
    }

    /// {@inheritDoc}
    /// @return The unix time in milliseconds when the ping was received
    @Override
    public long ping() { return System.currentTimeMillis(); }
}