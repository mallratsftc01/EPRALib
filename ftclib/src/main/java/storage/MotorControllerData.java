package storage;

/**A record that stores data from a MotorController.
 *<p></p>
 * Queer Coded by Striker-909.*/
public record MotorControllerData(long time, String address, double power, int position, int target, double velocity, double targetVelocity) {}