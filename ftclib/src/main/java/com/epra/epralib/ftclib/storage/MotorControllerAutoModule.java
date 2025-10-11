package com.epra.epralib.ftclib.storage;

/**A record that stores instructions for a MotorController during auto.
 *<p>
 *     JSON Format:
 *     <pre><code>
 *         {
 *             id: string,
 *             target: int,
 *             tolerance: float,
 *             maxPower: float,
 *             weight: float
 *         }
 *     </code></pre>
 *</p>
 * Queer Coded by Striker-909.
 * @param id The string id of the MotorController this module contains instructions for.
 * @param target The target in motor ticks of the MotorController.
 * @param tolerance The tolerance for reaching the target as a double between 0.0 and 1.0.
 * @param maxPower The maximum power the MotorController may reach. Capped at 1.0.
 * @param weight The weight, as a double between 1.0 and 0.0, that this module will contribute to moving to the next step of auto. A total weight of 1.0 or higher is necessary to move to the next step.*/
public record MotorControllerAutoModule(String id, int target, double tolerance, double maxPower, double weight) {}