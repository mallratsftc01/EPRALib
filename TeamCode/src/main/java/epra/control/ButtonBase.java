package epra.control;

public interface ButtonBase {

    boolean getBoolean();
    float getFloat();

    boolean getSingle();

    boolean getToggle();
    void setToggle(boolean t);
    void toggle();

    int getCounter();
    void setCounter(int c);
    void tickCounter(int i, int max);
}
