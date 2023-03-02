package frc.robot.subsystems;

public class Color {
    public int red;
    public int green;
    public int blue;

    // expects values from 0 to 255
    public Color(int r, int g, int b) {
        red = r;
        green = g;
        blue = b;
    }

    public static final Color whiteColor  = new Color(255, 255, 255);
    public static final Color redColor    = new Color(255, 0, 0);
    public static final Color greenColor  = new Color(0, 255, 0);
    public static final Color blueColor   = new Color(0, 0, 255);
    public static final Color yellowColor = new Color(255, 128, 0);
    public static final Color cyanColor   = new Color(0, 255, 255);
    public static final Color purpleColor = new Color(64, 0, 255);
    public static final Color off         = new Color(0, 0, 0);

}
