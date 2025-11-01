package com.epra.epralib.ftclib.storage.autonomous;

public class CRServoAutoModule {

    private final String id;
    private final double power;
    private final long time;

    /**
     * A record that stores instructions for a CRServo during auto.
     * <p>
     * JSON Format:
     * <pre><code>
     *         {
     *             "id": string,
     *             "power": float,
     *             "time": int
     *         } </code></pre>
     * </p>
     * Queer Coded by Striker-909.
     *
     * @param id    The string id of the CRServo this module contains instructions for.
     * @param power The power the CRServo will run at.
     * @param time  How long the CRServo should run for. (Making the time of the AutoStep longer than or equal to this time is recommended.)
     */
    public CRServoAutoModule(String id, double power, long time) {
        this.id = id;
        this.power = power;
        this.time = time;
    }

    public String id() { return id; }
    public double power() { return power; }
    public long time() { return time; }
}