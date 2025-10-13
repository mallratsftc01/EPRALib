package com.epra.epralib.ftclib.storage;

public class MotorControllerAutoModule {

    private final String id;
    private final int target;
    private final double tolerance;
    private final double maxPower;
    private final double power;
    private final double weight;

    /**
     * A record that stores instructions for a MotorController during auto.
     * <p>
     * JSON Format:
     * <pre><code>
     *         {
     *             "id": string,
     *             "target": int,
     *             "tolerance": float,
     *             "maxPower": float,
     *             "power": float,
     *             "weight": float
     *         } </code></pre>
     * </p>
     * Queer Coded by Striker-909.
     *
     * @param id        The string id of the MotorController this module contains instructions for.
     * @param target    The target in motor ticks of the MotorController.
     * @param tolerance The tolerance for reaching the target as a double between 0.0 and 1.0.
     * @param maxPower  The maximum power the MotorController may reach. Capped at 1.0.
     * @param power     The power to run the motor at. This will only be used if the tolerance is set to -1.0.
     * @param weight    The weight, as a double between 1.0 and 0.0, that this module will contribute to moving to the next step of auto. A total weight of 1.0 or higher is necessary to move to the next step.
     */
    public MotorControllerAutoModule(String id, int target, double tolerance, double maxPower, double power, double weight) {
        this.id = id;
        this.target = target;
        this.tolerance = tolerance;
        this.maxPower = maxPower;
        this.power = power;
        this.weight = weight;
    }

    public String id() { return id; }
    public int target() { return target; }
    public double tolerance() { return tolerance; }
    public double maxPower() { return maxPower; }
    public double power() { return power; }
    public double weight() { return weight; }
}