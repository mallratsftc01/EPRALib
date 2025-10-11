package com.epra.epralib.ftclib.storage;

/**A record that stores instructions for a CRServo during auto.
 *<p>
 *     JSON Format:
 *     <pre><code>
 *         {
 *             id: string,
 *             power: float,
 *             time: int
 *         }
 *     </code></pre>
 *</p>
 * Queer Coded by Striker-909.
 * @param id The string id of the CRServo this module contains instructions for.
 * @param power The power the CRServo will run at.
 * @param time How long the CRServo should run for. (Making the time of the AutoStep longer than or equal to this time is recommended.)
*/
public record CRServoAutoModule(String id, double power, long time) {}