package com.epra.epralib.ftclib.movement;

/**An interface for generalizing motors of all types.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public interface Motor {
    enum Direction {
        FORWARD,
        REVERSE
    }
    void setDirection(Direction direction);
    Direction getDirection();

    void setPower(double power);
    double getPower();

    boolean isEnabled();
    void setEnabled();
    void setDisabled();

    int getCurrentPosition();

    Object getSelf();
}
