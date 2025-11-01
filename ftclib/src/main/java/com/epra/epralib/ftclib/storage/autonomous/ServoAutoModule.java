package com.epra.epralib.ftclib.storage.autonomous;

public class ServoAutoModule {

    private final String id;
    private final double targetPosition;

    /**
     * A record that stores instructions for a Servo during auto.
     * <p>
     * JSON Format:
     * <pre><code>
     *         {
     *             "id": string,
     *             "targetPosition": float
     *         } </code></pre>
     * </p>
     * Queer Coded by Striker-909.
     *
     * @param id             The string id of the Servo this module contains instructions for.
     * @param targetPosition The target position of the Servo.
     */
    public ServoAutoModule(String id, double targetPosition) {
        this.id = id;
        this.targetPosition = targetPosition;
    }

    public String id() { return id; }
    public double targetPosition() { return targetPosition; }
}