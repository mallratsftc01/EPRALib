package storage;

public class SensorBooleanStorage {
    private boolean[] sensorValues = new boolean[0];
    /**Stores values from sensors in boolean form.
     *<p></p>
     *Queer Coded by Zachy K. If you use this class or a method from this class in its entirety, please make sure to give credit.
     *<p></p>
     *intended to store values from touch sensors. */
    public SensorBooleanStorage(boolean[] startValues) {setSensorValues(startValues);}

    /**Sets the stored values to match an array.*/
    public void setSensorValues(boolean[] setValues) {
        if (sensorValues.length != setValues.length) {sensorValues = new boolean[setValues.length];}
        for (int ii = 0; ii < setValues.length; ii++) {sensorValues[ii] = setValues[ii];}
    }
    /**Sets the value of a certain boolean in the stored array. If provided index is out of range, will return false.*/
    public boolean setSensorValue(boolean value, int index) {
        if (index < sensorValues.length) {sensorValues[index] = value;
            return true;} else {return false;}
    }
    /**Returns the stored boolean array.*/
    public boolean[] getSensorValues() {return sensorValues;}
    /**Returns a certain value from the stored array if successful. If provided index is out of range, will return false.*/
    public boolean getSensorValue(int index) {if (index < sensorValues.length) {return sensorValues[index];}
    else {return false;}
    }
}
