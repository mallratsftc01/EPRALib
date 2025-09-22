package com.epra.epralib.ftclib.movement;

import com.epra.epralib.ftclib.storage.PIDGains;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

/**Handles the processes of multiple PID loops.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class PIDController {

    private enum PIDVal {
        KP, KI, KD, I, SAVE_ERR, OUT
    }

    private static ArrayList<String> activeIds = new ArrayList<>();
    private static HashMap<String, HashMap<PIDVal, Double>> pidVals = new HashMap<>();
    private static HashMap<String, Supplier<Double>> errorSuppliers = new HashMap<>();

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
     * @param errorFunction A supplier function that returns the error in position.*/
    public static void addPID(String id, double kp, double ki, double kd, Supplier<Double> errorFunction) {
        if (saveTime == 0) {
            saveTime = System.currentTimeMillis();
        }
        activeIds.add(id);
        errorSuppliers.put(id, errorFunction);
        HashMap<PIDVal, Double> temp = new HashMap<>();
        temp.put(PIDVal.KP, kp);
        temp.put(PIDVal.KI, ki);
        temp.put(PIDVal.KD, kd);
        temp.put(PIDVal.I, 0.0);
        temp.put(PIDVal.SAVE_ERR, 0.0);
        temp.put(PIDVal.OUT, 0.0);
        pidVals.put(id, temp);
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
     * @param id The string id of the PID loop to reset.*/
    public static void reset(String id) {
        pidVals.get(id).replace(PIDVal.SAVE_ERR, 0.0);
        pidVals.get(id).replace(PIDVal.I, 0.0);
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

    public static boolean tune(String id, PIDGains pidGains) {
        if (!pidVals.containsKey(id)) {
            return false;
        }
        pidVals.get(id).replace(PIDVal.KP, pidGains.kp());
        pidVals.get(id).replace(PIDVal.KI, pidGains.ki());
        pidVals.get(id).replace(PIDVal.KD, pidGains.kd());
        return true;
    }

    public static boolean tune(String id, double kp, double ki, double kd) {
        if (!pidVals.containsKey(id)) {
            return false;
        }
        pidVals.get(id).replace(PIDVal.KP, kp);
        pidVals.get(id).replace(PIDVal.KI, ki);
        pidVals.get(id).replace(PIDVal.KD, kd);
        return true;
    }

    /**Updates all active PID loops.
     * @returns True if PID loops were successfully updated, false if no PID loops were active.*/
    public static boolean update() {
        if (activeIds.isEmpty() || saveTime == 0) {
            saveTime = System.currentTimeMillis();
            return false;
        }
        long timeElapsed = System.currentTimeMillis() - saveTime;
        saveTime = System.currentTimeMillis();

        for (String id : activeIds) {
            double currentError = errorSuppliers.get(id).get();
            if (pidVals.get(id).get(PIDVal.SAVE_ERR) == 0) {
                pidVals.get(id).replace(PIDVal.SAVE_ERR, currentError);
            }
            double p = currentError * pidVals.get(id).get(PIDVal.KP);
            double i = currentError * timeElapsed * pidVals.get(id).get(PIDVal.KI);
            double d =  ((currentError - pidVals.get(id).get(PIDVal.SAVE_ERR)) / timeElapsed) * pidVals.get(id).get(PIDVal.KD);
            pidVals.get(id).replace(PIDVal.SAVE_ERR, currentError);
            pidVals.get(id).replace(PIDVal.I, i);
            pidVals.get(id).replace(PIDVal.OUT, p + i + d);
        }
        return true;
    }

    /**Returns the most recent output of a specific PID loop.
     * @param id The string id of the PID output to return.
     * @return The most recent output of a specific PID loop.*/
    public static double get(String id) {
        return  pidVals.get(id).get(PIDVal.OUT);
    }
}
