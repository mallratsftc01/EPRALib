package com.epra.epralib.ftclib.movement.pid;

import com.epra.epralib.ftclib.storage.logdata.LogController;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/// Processes multiple PID loops simultaneously.
///
/// A PID loop uses Proportional, Integral, and Derivative elements of feedback to correct error smoothly.
/// [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller).
/// 
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public class PIDController {

    private static final ArrayList<String> activeIds = new ArrayList<>();
    private static final HashMap<String, PIDData> pidData = new HashMap<>();
    private static final Gson gson = new Gson();

    private static long saveTime = 0;

    /// Adds a new PID loop and activates it.
    /// @param id A tag that will identify this PID loop
    /// @param pidCoefficients An object with instructions to tune the PID loop
    /// @param errorSupplier A function that supplies the current error of the system that this PID loop will monitor
    public static void addPID(String id, PIDCoefficients pidCoefficients, Supplier<Double> errorSupplier) {
        addPID(id, pidCoefficients.p, pidCoefficients.i, pidCoefficients.d, errorSupplier);
    }

    /// Adds a new PID loop.
    /// @param id A tag that will identify this PID loop
    /// @param pidCoefficients A record with instructions to tune the PID loop
    /// @param errorSupplier A function that supplies the current error of the system that this PID loop will monitor
    /// @param active If this PID loop should start active
    public static void addPID(String id, PIDCoefficients pidCoefficients, Supplier<Double> errorSupplier, boolean active) {
        addPID(id, pidCoefficients, errorSupplier);
        if (!active) {
            idle(id);
        }
    }

    /// Adds a new PID loop and activates it.
    /// @param id A tag that will identify this PID loop
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    /// @param errorSupplier A function that supplies the current error of the system that this PID loop will monitor
    public static void addPID(String id, double kp, double ki, double kd, Supplier<Double> errorSupplier) {
        if (saveTime == 0) {
            saveTime = System.currentTimeMillis();
        }
        activeIds.add(id);
        pidData.put(id, new PIDData(kp, ki, kd, errorSupplier));
    }

    /// Adds a new PID loop and activates it.
    /// @param id A tag that will identify this PID loop
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    /// @param errorSupplier A function that supplies the current error of the system that this PID loop will monitor
    /// @param active If this PID loop should start active
    public static void addPID(String id, double kp, double ki, double kd, Supplier<Double> errorSupplier, boolean active) {
        addPID(id, kp, ki, kd, errorSupplier);
        if (!active) {
            idle(id);
        }
    }

    /// Fetches all [PIDCoefficients] from the given `filename`.
    ///
    /// Will return `null` and log an error if the `filename` cannot be found.
    /// @param filename The filename of the `JSON` file with the PID coefficients
    /// @return A hashmap of all PID coefficients in the file, indexed by their `id`
    public static HashMap<String, PIDCoefficients> getPIDsFromFile(String filename) {
        try (FileReader reader = new FileReader(AppUtil.getInstance().getSettingsFile(filename))) {
            HashMap<String, PIDCoefficients> temp = gson.fromJson(reader, new TypeToken<HashMap<String, PIDCoefficients>>() {}.getType());
            LogController.logInfo("Successfully got all PIDs from " + filename);
            return temp;
        } catch (Exception e) {
            LogController.logError("Trouble fetching PID gains from " + filename +
                    ". Error: " + e.getMessage());
            return null;
        }
    }

    /// Tunes all PIDs given a [HashMap] that maps PID `ids` to the [PIDCoefficients] that PID should be tuned with.
    /// @param pidGainsMap A hash map from a PID's `id` to the pid gains record to tune it with
    /// @return If a PID exists for every `id` in the `pidGainsMap`
    public static boolean tuneAllPIDs(HashMap<String, PIDCoefficients> pidGainsMap) {
        boolean out = true;
        for (String id : pidGainsMap.keySet()) {
            if (pidData.containsKey(id)) {
                pidData.get(id).tune(pidGainsMap.get(id));
            } else {
                out = false;
            }
        }
        return out;
    }

    /// Fetches all [PIDCoefficients] from the given `filename` and tunes all PIDs accordingly.
    /// @param filename The filename of the `JSON` file with the PID gains
    /// @return If the `filename` could be found
    public static boolean tuneAllPIDsFromFile(String filename) {
        HashMap<String, PIDCoefficients> pidGainsMap = getPIDsFromFile(filename);
        if (pidGainsMap == null) { return false; }
        tuneAllPIDs(pidGainsMap);
        return true;
    }

    /// Returns if a PID loop with the provided `id` has been created.
    /// @param id The `id` of the PID loop to check for
    /// @return If a PID loop with the provided `id` exists
    public static boolean hasPID(String id) {
        return pidData.containsKey(id) && pidData.get(id) != null;
    }

    /// Resets the variable values of the PID loop with the provided `id`.
    /// @param id The `id` of the PID loop to reset
    /// @return If a PID loop with the provided `id` exists
    public static boolean reset(String id) {
        if (!hasPID(id)) {
            return false;
        }
        pidData.get(id).saveError = 0.0;
        pidData.get(id).i = 0.0;
        return true;
    }

    /// Deactivates the PID loop with the provided `id`.
    /// 
    /// Idle PID loops will not be updated every time [#update()] is called.
    /// @param id The `id` of the PID loop to idle
    /// @return If a PID loop with the provided `id` was active before it was idled
    public static boolean idle(String id) {
        reset(id);
        return activeIds.remove(id);
    }

    /// Activates the PID loop with the provided `id`.
    /// 
    /// Active PID loops will be updated everytime [#update()] is called.
    /// @param id The `id` of the PID loop to activate
    /// @return If a PID loop with the provided `id` was idle before it was activated
    public static boolean activate(String id) {
        if (activeIds.contains(id) || !hasPID(id)) {
            return false;
        }
        activeIds.add(id);
        return true;
    }

    /// Checks if the PID loop with the provided `id` exists and is active.
    /// 
    /// Active PID loops will be updated everytime [#update()] is called.
    /// @param id The `id` of the PID loop to be checked
    /// @return If the PID loop with the provided `id` exists and is active
    public static boolean isActive(String id) {
        return hasPID(id) && activeIds.contains(id);
    }

    /// Modifies the [PIDCoefficients] for the PID loop with the provided `id`.
    /// @param id The `id` of the PID loop to be modified
    /// @param pidCoefficients A record with instructions to tune the PID loop
    /// @return If the PID loop with the provided `id` exists
    public static boolean tune(String id, PIDCoefficients pidCoefficients) {
        if (!hasPID(id)) {
            return false;
        }
        pidData.get(id).tune(pidCoefficients);
        return true;
    }
    /// Modifies the gain constants for the PID loop with the provided `id`.
    /// @param id The `id` of the PID loop to be modified
    /// @param kp The `p` constant
    /// @param ki The `i` constant
    /// @param kd The `d` constant
    /// @return If the PID loop with the provided `id` exists
    public static boolean tune(String id, double kp, double ki, double kd) {
        if (!pidData.containsKey(id) || pidData.get(id) == null) {
            return false;
        }
        pidData.get(id).tune(kp, ki, kd);
        return true;
    }

    /// Updates all active PID loops.
    /// @return If there were any active PID loops to update
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

    /// Retrieve the most recent output of the PID loop with the provided `id`.
    /// 
    /// Will return `NaN` if no PID loop with the provided `id` exists.
    /// @param id The `id` of the PID loop to be retrieved
    /// @return The most recent output of the PID loop with the provided `id`
    public static double get(String id) {
        return (hasPID(id)) ? pidData.get(id).output : Double.NaN;
    }
}
