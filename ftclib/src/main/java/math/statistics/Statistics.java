package math.statistics;

import java.util.ArrayList;

/**A class that adds many statistical functions for many uses.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Statistics {

    public Statistics() {}

    /**@param set A set of values.
     * @return The average value of the set.*/
    public static double average(double[] set) {
        double v = 0.0;
        for (double d : set) { v += d; }
        return v / (double) set.length;
    }
    /**Finds the standard deviation of the mean for a set of values.
     * @param set A set of values.
     * @return the Standard Deviation of the set.*/
    public static double standardDeviation(double[] set) {
        double mean = average(set);
        double SS = 0;
        for (double x : set) { SS += Math.pow(x - mean, 2); }
        return Math.sqrt(SS / (set.length - 1));
    }

    /**Finds the derivative array of an array.
     * @param set The array to find the derivative of.
     * @return The array of the derivative.*/
    public static double[] differentiate(double[] set) {
        double[] r = new double[set.length];
        r[0] = set[1] - set[0];
        r[set.length - 1] = r[set.length - 1] - r[set.length - 2];
        for (int i = 1; i < set.length - 1; i++) {
            r[i] = (set[i + 1] - set[i - 1]) / 2;
        }
        return r;
    }
    /**Finds the derivative array of an array.
     * @param set The array to find the derivative of.
     * @return The array of the derivative.*/
    public static double[] differentiate(double[] set, long[] time) {
        double[] r = new double[set.length];
        r[0] = set[1] - set[0];
        r[set.length - 1] = r[set.length - 1] - r[set.length - 2];
        for (int i = 1; i < set.length - 1; i++) {
            r[i] = (set[i + 1] - set[i - 1]) / (time[i + 1] - time[i - 1]);
        }
        return r;
    }

    /**Converts an ArrayList of doubles to an Array.
     * @param set An ArrayList of doubles.
     * @return An array of doubles.*/
    public static double[] toSet(ArrayList<Double> set) {
        double[] r = new double[set.size()];
        for (int i = 0; i < r.length; i++) { r[i] = set.get(i).doubleValue(); }
        return r;
    }
    /**Converts an ArrayList of longs to an Array.
     * @param set An ArrayList of longs.
     * @return An array of longs.*/
    public static long[] toTime(ArrayList<Long> set) {
        long[] r = new long[set.size()];
        for (int i = 0; i < r.length; i++) { r[i] = set.get(i).longValue(); }
        return r;
    }
}
