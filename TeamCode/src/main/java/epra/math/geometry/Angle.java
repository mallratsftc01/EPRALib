package epra.math.geometry;

/**Stores an Angle value.
 *<p></p>
 *Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class Angle {

    private double radian;

    /**Stores an Angle value.
     *<p></p>
     * @param radian Radian value to set stored value to.*/
    public Angle(float radian) { this.radian = radian; }
    /**Stores an Angle value.
     *<p></p>
     * @param degree Degree value to set stored value to.*/
    public Angle(double degree) { this.radian = (Math.PI / 180.0) * degree; }
    /**Stores an Angle value.*/
    public Angle() { this.radian = 0.0; }

    /**@param radian Radian value to set stored value to.*/
    public void setRadian(double radian) { this.radian = radian; }
    /**@param degree Degree value to set stored value to.*/
    public void setDegree(double degree) { this.radian = (Math.PI / 180.0) * degree; }

    /**@return The radian stored value;*/
    public double getRadian() { return this.radian;}
    /**@return The degree stored value;*/
    public double getDegree() { return (180.0 / Math.PI) * this.radian; }
}