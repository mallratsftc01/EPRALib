package com.epra.epralib.ftclib.storage.logdata;

import java.util.HashMap;

/// An interface to be implemented by any class to facilitate the logging of data with [LogController].
///
/// Queer Coded by Striker-909.
/// If you use this class or a method from this class in its entirety, please make sure to give credit.
public interface DataLogger {
    /// Retrieves the file path for the log file for [LogController] to use for this logger,
    /// relative to the base `log` directory.
    /// @return The relative file path for the logs from this logger
    String getLogPath();

    /// Retrieves the data to be logged from this logger as a [HashMap].
    /// @return A hash map with a data snapshot of this logger
    HashMap<String, Double> logData();

    /// Updates any values that need to be updated to log properly.
    /// @return If log values were successfully updated
    boolean updateLog();

    /// Returns the ping and the time the ping was received.
    /// @return The unix time in milliseconds when the ping was received
    long ping();
}
