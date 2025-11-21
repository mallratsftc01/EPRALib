package com.epra.epralib.ftclib.movement;

import com.epra.epralib.ftclib.storage.logdata.PIDData;
import com.epra.epralib.ftclib.storage.initialization.PIDGains;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

/**Handles the processes of multiple PID loops.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class PIDController {

    private static final ArrayList<String> activeIds = new ArrayList<>();
    private static final HashMap<String, PIDData> pidData = new HashMap<>();

    private static long saveTime = 0;

    /**Adds a new PID loop and automatically activates it.
     * @param id The string id for the PID loop.
     * @param pidGains The gains for the PID loop.
     * @param errorSupplier A supplier that returns the error in position.*/
    public static void addPID(String id, PIDGains pidGains, Supplier<Double> errorSupplier) {
        addPID(id, pidGains.kp(), pidGains.ki(), pidGains.kd(), errorSupplier);
    }

    /**Adds a new PID loop.
     * @param id The string id for the PID loop.
     * @param pidGains The gains for the PID loop.
     * @param errorSupplier A supplier that returns the error in position.
     * @param active Will activate the PID if true, will idle otherwise.*/
    public static void addPID(String id, PIDGains pidGains, Supplier<Double> errorSupplier, boolean active) {
        addPID(id, pidGains, errorSupplier);
        if (!active) {
            idle(id);
        }
    }

    /**Adds a new PID loop and automatically activates it.
     * @param id The string id for the PID loop.
     * @param kp The p gain for the PID loop.
     * @param ki The i gain for the PID loop.
     * @param kd the d gain for the PID loop.
     * @param errorSupplier A supplier that returns the error in position.*/
    public static void addPID(String id, double kp, double ki, double kd, Supplier<Double> errorSupplier) {
        if (saveTime == 0) {
            saveTime = System.currentTimeMillis();
        }
        activeIds.add(id);
        pidData.put(id, new PIDData(kp, ki, kd, errorSupplier));
    }

    /**Adds a new PID loop.
     * @param id The string id for the PID loop.
     * @param kp The p gain for the PID loop.
     * @param ki The i gain for the PID loop.
     * @param kd the d gain for the PID loop.
     * @param errorSupplier A supplier that returns the error in position.
     * @param active Will activate the PID if true, will idle otherwise.*/
    public static void addPID(String id, double kp, double ki, double kd, Supplier<Double> errorSupplier, boolean active) {
        addPID(id, kp, ki, kd, errorSupplier);
        if (!active) {
            idle(id);
        }
    }

    /**Resets the non-gain values for a specific PID loop.
     * @param id The string id of the PID loop to reset.
     * @return True if the specified PID loop exists, false if not.*/
    public static boolean reset(String id) {
        if (!pidData.containsKey(id) || pidData.get(id) == null) {
            return false;
        }
        pidData.get(id).saveError = 0.0;
        pidData.get(id).i = 0.0;
        return true;
    }

    /**Idles a specific PID loop so that computation power will not be spent to run it. Also resets that PID loop.
     * @param id The string id of the PID loop to idle.
     * @return True if the PID loop was active, false if not.*/
    public static boolean idle(String id) {
        reset(id);
        return activeIds.remove(id);
    }

    /**Activates a specific PID loop.
     * @param id The string id of the PID loop to activate.
     * @return True if the PID loop was idle, false if not.*/
    public static boolean activate(String id) {
        if (activeIds.contains(id)) {
            return false;
        }
        activeIds.add(id);
        return true;
    }

    /**Returns if a specific PID loop is active.
     * @return True if the PID loop is active, false if not.*/
    public static boolean isActive(String id) {
        return activeIds.contains(id);
    }

    /**Sets the PID gains of the specified PID loop to the specified values.
     * @param id The string id of the PID loop to tune.
     * @param pidGains The gains for the PID loop.
     * @return True if the specified PID loop exists, false if not.*/
    public static boolean tune(String id, PIDGains pidGains) {
        if (!pidData.containsKey(id)) {
            return false;
        }
        pidData.get(id).tune(pidGains);
        return true;
    }
    /**Sets the PID gains of the specified PID loop to the specified values.
     * @param id The string id of the PID loop to tune.
     * @param kp The p gain for the PID loop.
     * @param ki The i gain for the PID loop.
     * @param kd The d gain for the PID loop.
     * @return True if the specified PID loop exists, false if not.*/
    public static boolean tune(String id, double kp, double ki, double kd) {
        if (!pidData.containsKey(id) || pidData.get(id) == null) {
            return false;
        }
        pidData.get(id).tune(kp, ki, kd);
        return true;
    }

    /**Updates all active PID loops.
     * @return True if PID loops were successfully updated, false if no PID loops were active.*/
    public static boolean update() {
        if (activeIds.isEmpty() || saveTime == 0) {
            saveTime = System.currentTimeMillis();
            return false;
        }
        long timeElapsed = System.currentTimeMillis() - saveTime;
        saveTime = System.currentTimeMillis();

        for (String id : activeIds) {
            if (pidData.get(id) == null) {
                continue;
            }
            double currentError = pidData.get(id).getError();
            if (pidData.get(id).saveError == 0.0 && currentError != 0.0) {
                pidData.get(id).saveError = currentError;
            }
            double p = currentError * pidData.get(id).kp;
            double i = currentError * timeElapsed * pidData.get(id).ki;
            double d =  ((currentError - pidData.get(id).saveError) / timeElapsed) * pidData.get(id).kd;
            pidData.get(id).saveError = currentError;
            pidData.get(id).i = i;
            pidData.get(id).output = p + i + d;
        }
        return true;
    }

    /**Returns the most recent output of a specific PID loop.
     * @param id The string id of the PID output to return.
     * @return The most recent output of a specific PID loop.*/
    public static double get(String id) {
        return  pidData.get(id).output;
    }
}
