package com.epra.epralib.ftclib.movement;

import java.util.ArrayList;

import com.epra.epralib.ftclib.math.statistics.Statistics;

public class PIDTuner {

    private double startError = 0;
    private ArrayList<Double> errorBuffer = new ArrayList<>();
    private ArrayList<Long> timeBuffer = new ArrayList<>();

    public PIDTuner() {}

    public void logPID(double error, double power) {
        if (startError == 0) { startError = error; }
        if (Math.abs(power) < 1.0) {
            errorBuffer.add(error);
            timeBuffer.add(System.currentTimeMillis());
        }
    }

    public double pFitness() { return Math.abs((startError - errorBuffer.get(errorBuffer.size() - 1)) / startError); }

    public double dFitness() {
        double[] errorSet = Statistics.toSet(errorBuffer);
        long[] timeSet = Statistics.toTime(timeBuffer);
        double[] derivative_2 = Statistics.differentiate(Statistics.differentiate(errorSet, timeSet), timeSet);
        return Math.abs(Statistics.standardDeviation(derivative_2) / errorSet[0]);
    }

    public void reset() {
        startError = 0;
        errorBuffer.clear();
        timeBuffer.clear();
    }
}
