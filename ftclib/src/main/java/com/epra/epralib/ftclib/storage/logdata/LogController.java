package com.epra.epralib.ftclib.storage.logdata;

import android.annotation.SuppressLint;
import com.google.gson.Gson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Supplier;

import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

/// Handles a central log and individual logs for specific [DataLogger]s.
///
/// [#init()] will create the log directory at `sdcard/FIRST/settings/logs/DATE`.
/// The `DATE` is in the form `ddMMyyyy:HH:mm`.
/// The main log is `main.log` within that directory.
/// Various levels of messages can be written to the central log.
///
/// [#addLogger(DataLogger)] will add a data logger, whose log can be found at
/// `TYPE_ID.json` in the log directory.
/// [#logData()] will update and log data from all data loggers.
public class LogController {
    private static String DATE;
    private static String LOG_PATH;

    private static final Logger mainLogger = Logger.getLogger(LogController.class.getName());

    private static final Gson gson = new Gson();
    private static long startTime;
    private final static HashMap<String, Supplier<HashMap<String, Double>>> logData = new HashMap<>();
    private final static HashMap<String, FileWriter> files = new HashMap<>();
    private final static HashMap<String, Supplier<Boolean>> updateLog = new HashMap<>();
    private final static HashMap<String, Supplier<Long>> ping = new HashMap<>();

    private static final Timer timer = new Timer();

    /// Initializes the log controller, creating the directory for log files and a main log file.
    ///
    /// **The log controller MUST be initialized for any other methods to function.**
    ///
    /// Will abort if any [IOException] occur.
    /// These errors will not be able to be logged as the main log will not yet be created.
    /// The log directory will be at `sdcard/FIRST/settings/logs/DATE`.
    /// The `DATE` is in the form `ddMMyyyy:HH:mm`.
    /// The main log is `main.log` within that directory.
    /// @return If the directory and main log were successfully created
    public static boolean init() {
        @SuppressLint("SimpleDateFormat") SimpleDateFormat ft = new SimpleDateFormat("ddMMyyyy:HH:mm");
        DATE = ft.format(new Date());
        LOG_PATH = "logs/" + DATE + "/";
        startTime = System.currentTimeMillis();

        File logDir = AppUtil.getInstance().getSettingsFile(LOG_PATH);
        if (!logDir.exists()) { if (!logDir.mkdirs()) { return false; } }
        try {
            FileHandler mainLogFile = new FileHandler(AppUtil.getInstance().getSettingsFile(LOG_PATH + "main.log").getAbsolutePath());
            mainLogger.addHandler(mainLogFile);
            mainLogFile.setFormatter(new SimpleFormatter());
        } catch (IOException e) {
            return false;
        }
        mainLogger.log(Level.INFO, "Log Controller Initialization Complete");
        return true;
    }

    /// Logs a message to `main.log` at [Level#INFO].
    ///
    /// To be used for small updates.
    /// @param msg The message to be logged
    ///
    /// @see #init
    public static void logInfo(String msg) { mainLogger.log(Level.INFO, msg); }
    /// Logs a message to `main.log` at [Level#WARNING].
    ///
    /// To be used for non-critical errors.
    /// @param msg The message to be logged
    ///
    /// @see #init
    public static void logWarning(String msg) { mainLogger.log(Level.WARNING, msg); }
    /// Logs a message to `main.log` at [Level#SEVERE].
    ///
    /// To be used for critical errors.
    /// @param msg The message to be logged
    ///
    /// @see #init
    public static void logError(String msg) { mainLogger.log(Level.SEVERE, msg); }

    /// Adds a [DataLogger] for the log controller to log data from.
    ///
    /// What data will be logged is set in the construction of that data logger.
    /// Data will be logged in a `JSON` file in the log directory, as `TYPE_ID.json`.
    /// Data is logged from all data loggers when [#logData()] is called.
    /// @param dataLogger A data logger
    /// @return If the data logger was unique and successfully added
    ///
    /// @see #init()
    public static boolean addLogger(DataLogger dataLogger) {
        String filename = dataLogger.getLogPath();
        if (logData.containsKey(filename)) { return false; }
        try {
            files.put(filename, new FileWriter(AppUtil.getInstance().getSettingsFile(LOG_PATH + filename), true));
            files.get(filename).write("[");
        } catch (IOException e) {
            mainLogger.log(Level.SEVERE, "Failed to create log file " + filename);
            return false;
        }
        logData.put(filename, dataLogger::logData);
        updateLog.put(filename, dataLogger::updateLog);
        ping.put(filename, dataLogger::ping);
        return true;
    }

    /// Updates and logs data from all [DataLogger]s to their respective `JSON` files.
    ///
    /// @return If all data was successfully logged
    ///
    /// @see #fixedRateLog(double)
    /// @see #closeLogs()
    /// @see #init()
    public static boolean logData() {
        boolean result = true;
        mainLogger.log(Level.INFO, "Logging data...");
        for (String filename : logData.keySet()) {
            if (!updateLog.get(filename).get()) {
                mainLogger.log(Level.WARNING, "Could not update data for " + filename);
                result = false;
                continue;
            }
            HashMap<String, Double> data = logData.get(filename).get();
            data.put("time", (System.currentTimeMillis() - startTime) / 1000.0);
            try {
                files.get(filename).write("\n" + gson.toJson(data) + ",");
            } catch (IOException e) {
                mainLogger.log(Level.WARNING, "Failed to write to log file " + filename);
                result = false;
            }
        }
        if (result) mainLogger.log(Level.INFO, "All data logged successfully");
        return result;
    }
    /// Calls [#logData()] on a regular interval.
    ///
    /// Will stop logging when [#closeLogs()] is called.
    /// @param rate Delay between calls in seconds
    ///
    /// @see #init()
    public static void fixedRateLog(double rate) {
        logData();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                fixedRateLog(rate);
            }
        }, (long) (rate * 1000));
    }
    /// Closes all [DataLogger]s and their respective `JSON` files.
    ///
    /// @return If all logs were successfully closed
    ///
    /// @see #logData()
    /// @see #fixedRateLog(double)
    /// @see #init()
    public static boolean closeLogs() {
        mainLogger.log(Level.INFO, "Closing logs...");
        timer.cancel();
        for (String filename : files.keySet()) {
            try {
                FileWriter file = files.get(filename);
                file.write("]");
                file.close();
            } catch (IOException e) {
                mainLogger.log(Level.SEVERE, "Failed to close log " + filename);
                return false;
            }
        }
        mainLogger.log(Level.INFO, "All logs closed successfully");
        return true;
    }

    /// Pings all [DataLogger]s and logs their latencies.
    public static void ping() {
        for (String filename :  logData.keySet()) {
            long t = System.currentTimeMillis();
            long p = ping.get(filename).get();
            mainLogger.log(Level.INFO, filename + "ping: " + (p-t) + " ms");
        }
    }
}
