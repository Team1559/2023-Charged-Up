package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardPIDTuner {
    // Flags determine which terms are enabled, user doesn't need all of them
    public static final int FLAG_KP = 1 << 0;
    public static final int FLAG_KI = 1 << 1;
    public static final int FLAG_KD = 1 << 2;
    public static final int FLAG_KF = 1 << 3;
    public static final int FLAG_KG = 1 << 4;
    public static final int FLAG_KV = 1 << 5;
    public static final int FLAG_KA = 1 << 6;

    public static final int FLAGS_PID = FLAG_KP | FLAG_KI | FLAG_KD;
    public static final int FLAGS_ALL;

    private static final String KEY_KP = " kP";
    private static final String KEY_KI = " kI";
    private static final String KEY_KD = " kD";
    private static final String KEY_KF = " kF";
    private static final String KEY_KG = " kG";
    private static final String KEY_KV = " kV";
    private static final String KEY_KA = " kV";

    private static final int[]    FLAGS = { FLAG_KP, FLAG_KI, FLAG_KD, FLAG_KF,
            FLAG_KG, FLAG_KV };
    private static final String[] KEYS  = { KEY_KP, KEY_KI, KEY_KD, KEY_KF,
            KEY_KG, KEY_KV };

    static {
        int allFlags = 0;
        for (int i = 0; i < FLAGS.length; i++) {
            allFlags |= FLAGS[i];
        }
        FLAGS_ALL = allFlags;
    }

    private final String prefix;
    private final int    coefficients;

    public DashboardPIDTuner(String prefix, int coefficients) {
        this.prefix = prefix;
        this.coefficients = coefficients;
        for (int i = 0; i < FLAGS.length; i++) {
            if (isEnabled(FLAGS[i])) {
                SmartDashboard.putNumber(getKey(KEYS[i]), 0);
            }
        }
    }

    private boolean isEnabled(int flag) {
        return (coefficients & flag) == flag;
    }

    private String getKey(String suffix) {
        return prefix + suffix;
    }

    public double getKP() {
        return SmartDashboard.getNumber(getKey(KEY_KP), 0);
    }

    public double getKI() {
        return SmartDashboard.getNumber(getKey(KEY_KI), 0);
    }

    public double getKD() {
        return SmartDashboard.getNumber(getKey(KEY_KD), 0);
    }

    public double getKF() {
        return SmartDashboard.getNumber(getKey(KEY_KF), 0);
    }

    public double getKG() {
        return SmartDashboard.getNumber(getKey(KEY_KG), 0);
    }

    public double getKV() {
        return SmartDashboard.getNumber(getKey(KEY_KV), 0);
    }

    public double getKA() {
        return SmartDashboard.getNumber(getKey(KEY_KA), 0);
    }
}
