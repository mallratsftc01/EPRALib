package epra.math.statistics;

import java.util.ArrayList;
import java.util.function.BiFunction;

/**A rolling average of values stored in a buffer.
 * <p>
 * This class is intended for use sensors that give sometimes unreliable outputs. This average should flatten out any outliers.
 * The average can be biased towards more recent values so that actual movement is preserved.
 * <p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class RollingAverage {

    /**Enum containing all types of bias the rolling average can use and the corresponding methods for those bias types.*/
    public enum Bias {
        SIGMOID(RollingAverage::sigmoidBias),
        REVERSE_SIGMOID(RollingAverage::reverseSigmoidBias),
        LINEAR(RollingAverage::linearBias),
        REVERSE_LINEAR(RollingAverage::reverseLinearBias),
        FLAT(RollingAverage::flatBias);

        BiFunction<Integer, Integer, Double> use;

        Bias(BiFunction<Integer, Integer, Double> f) { this.use = f; }
    }

    public enum Threshold {
        NO_CHANGE,
        NO_CHANGE_ZERO,
        NO_CHANGE_AVERAGE,
        NONE,
    }

    private ArrayList<Double> buffer;
    private ArrayList<Double> averageBuffer;
    private int bufferSize;
    private Bias biasType;
    private int autoClearThreshold;
    private Threshold thresholdType;

    /**A rolling average of values stored in a buffer.
     *<p></p>
     * This class is intended for use sensors that give sometimes unreliable outputs. This average should flatten out any outliers.
     * The average can be biased towards more recent values so that actual movement is preserved.
     * @param bufferSize Size of the buffer.
     * @param biasType Type of bias to use in averages.*/
    public RollingAverage(int bufferSize, Bias biasType) {
        buffer = new ArrayList<Double>();
        averageBuffer = new ArrayList<Double>();
        this.bufferSize = bufferSize;
        this.biasType = biasType;
        this.autoClearThreshold = 0;
        this.thresholdType = Threshold.NONE;
    }

    /**A rolling average of values stored in a buffer.
     *<p></p>
     * This class is intended for use sensors that give sometimes unreliable outputs. This average should flatten out any outliers.
     * The average can be biased towards more recent values so that actual movement is preserved.
     * @param bufferSize Size of the buffer.
     * @param biasType Type of bias to use in averages.
     * @param thresholdType The type of auto clear threshold to use.
     * @param threshold the threshold value.*/
    public RollingAverage(int bufferSize, Bias biasType, Threshold thresholdType, int threshold) {
        buffer = new ArrayList<Double>();
        averageBuffer = new ArrayList<Double>();
        this.bufferSize = bufferSize;
        this.biasType = biasType;
        this.autoClearThreshold = threshold;
        this.thresholdType = thresholdType;
    }

    /**@param bufferSize New size of the buffer.*/
    public void setBufferSize(int bufferSize) { this.bufferSize = bufferSize; }
    /**@param biasType New type of bias to use in averages.*/
    public void setBiasType(Bias biasType) { this.biasType = biasType; }
    /**@param thresholdType The type of auto clear threshold to use.
     * @param threshold the threshold value.*/
    public void setThreshold(Threshold thresholdType, int threshold) {
        this.autoClearThreshold = threshold;
        this.thresholdType = thresholdType;
    }

    /**Creates a sigmoid bias multiplier between 0.0 and 2.0 so that more recent values are given more weight than older values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double sigmoidBias(int total, int recency) { return 2 / (1 + Math.pow(Math.E, ((double) (total - 1) / 2.0) - (double) (total - recency))); }
    /**Creates a sigmoid bias multiplier between 0.0 and 2.0 so that older values are given more weight than more recent values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double reverseSigmoidBias(int total, int recency) { return 2 / (1 + Math.pow(Math.E, ((double) (total - 1) / 2.0) - (double) (recency - total))); }
    /**Creates a linear bias multiplier between 0.0 and 2.0 so that more recent values are given more weight than older values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double linearBias(int total, int recency) { return 2.0 * ((double) recency / (double) total); }
    /**Creates a linear bias multiplier between 0.0 and 2.0 so that older values are given more weight than more recent values.
     * @param total The total number of values in the buffer.
     * @param recency A value's index in the buffer (with higher being more recent).
     * @return A double multiplier between 0.0 and 2.0.*/
    public static double reverseLinearBias(int total, int recency) { return 2.0 * ((double) (total - recency) / (double) total); }
    /**Creates a flat bias of 1.0, params are not used.
     * @return 1.0*/
    public static double flatBias(int total, int recency) { return 1.0; }

    /**Determines if recent values have all been the same, suggesting that the buffer should be cleared.
     * @param buffer The buffer of values.
     * @param threshold The threshold to be measured against.
     * @return True if the buffer should be cleared.*/
    public static boolean noChangeThreshold(Double[] buffer, int threshold) {
        double save = buffer[buffer.length - 1];
        for (int i = 2; i < threshold; i++) { if (buffer[buffer.length - i] != save) { return true; } }
        return false;
    }
    /**Determines if recent values have all been the same, suggesting that the buffer should be cleared.
     * @param buffer The buffer of values.
     * @param threshold The threshold to be measured against.
     * @param target The target value to test for.
     * @return True if the buffer should be cleared.*/
    public static boolean noChangeThreshold(Double[] buffer, int threshold, double target) {
        for (int i = 2; i < threshold; i++) { if (buffer[buffer.length - i] != target) { return true; } }
        return false;
    }
    /**Determines if recent averages have all been the same, suggesting that the buffer should be cleared.
     * @param buffer The buffer of averages.
     * @return True if the buffer should be cleared.*/
    public static boolean averageThreshold(Double[] buffer) {
        double save = buffer[0];
        for (int i = 1; i < buffer.length; i++) { if (buffer[i] != save) { return false; } }
        return true;
    }

    /**@return The weighted average value of the buffer.*/
    public double getAverage() {
        double[] set = new double[buffer.size()];
        for (int i = 0; i < set.length; i++) { set[i] = buffer.get(i) * biasType.use.apply(set.length, i); }
        return Statistics.average(set);
    }
    /**@param value A value to add to the buffer.
     * @return The weighted average value of the buffer.*/
    public double addValue(double value) {
        buffer.add(value);
        while (buffer.size() > bufferSize) { buffer.remove(0); }
        averageBuffer.add(getAverage());
        while (averageBuffer.size() > autoClearThreshold) { averageBuffer.remove(0); }
        if (buffer.size() > autoClearThreshold) {
            if (switch (thresholdType) {
                case NO_CHANGE -> noChangeThreshold((Double[]) buffer.toArray(), autoClearThreshold);
                case NO_CHANGE_ZERO -> noChangeThreshold((Double[]) buffer.toArray(), autoClearThreshold, 0);
                case NO_CHANGE_AVERAGE -> averageThreshold((Double[]) averageBuffer.toArray());
                case NONE -> false;
            } ) {
                buffer.clear();
            }
        }
        return getAverage();
    }
    /**@param set Set of values to add to the buffer.
     * @return The weighted average value of the buffer.*/
    public double addValue(double[] set) {
        for (double d : set) {
            buffer.add(d);
            while (buffer.size() > bufferSize) { buffer.remove(0); }
            averageBuffer.add(getAverage());
        }
        if (buffer.size() > autoClearThreshold) {
            if (switch (thresholdType) {
                case NO_CHANGE -> noChangeThreshold((Double[]) buffer.toArray(), autoClearThreshold);
                case NO_CHANGE_ZERO -> noChangeThreshold((Double[]) buffer.toArray(), autoClearThreshold, 0);
                case NO_CHANGE_AVERAGE -> averageThreshold((Double[]) averageBuffer.toArray());
                case NONE -> false;
            } ) {
                buffer.clear();
            }
        }
        return getAverage();
    }

    /**@return The buffer as an array.*/
    public double[] toArray() {
        double[] a = new double[buffer.size()];
        for (int i = 0; i < buffer.size(); i++) { a[i] = buffer.get(i); }
        return a;
    }

    /**@return A new RollingAverage of the derivative of this RollingAverage.*/
    public RollingAverage getDerivative() {
        RollingAverage r = new RollingAverage(bufferSize, biasType, thresholdType, autoClearThreshold);
        if (buffer.size() > 2) {
            r.addValue(Statistics.differentiate(this.toArray()));
        }
        return r;
    }
}