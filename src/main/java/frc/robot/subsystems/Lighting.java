package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase {
    private static final Color[] colors = { Color.purpleColor, Color.yellowColor, Color.blueColor, Color.redColor};

    private final int length;
    private int       clock;

    private AddressableLEDBuffer buffer;
    private AddressableLED       ledStrip;

    public Lighting(int portNum, int length) {
        this.length = length;

        buffer = new AddressableLEDBuffer(length);
        ledStrip = new AddressableLED(portNum);
        ledStrip.setLength(length);
        ledStrip.start();

        
    }

    public void setColor(Color color) {
        for (int i = 0; i < length; i++) {
            setColor(i, color);
        }
    }

    public void setColor(int index, Color color) {
        buffer.setRGB(index, color.red, color.green, color.blue);
    }

    // private DigitalOutput greenDO;
    // private DigitalOutput blueDO;
    // private final double PULSE_LENGTH_MS = 5.005;

    // public Lighting(int pwmRed, int pwmGreen, int pwmBlue) {
    // redPWM = new PWM(pwmRed);
    // greenPWM = new PWM(pwmGreen);
    // bluePWM = new PWM(pwmBlue);
    // redDO = new DigitalOutput(pwmRed);
    // redDO.enablePWM(0);
    // greenDO = new DigitalOutput(pwmGreen);
    // greenDO.enablePWM(0);
    // blueDO = new DigitalOutput(pwmBlue);
    // blueDO.enablePWM(0);

    // redPWM.setBounds(PULSE_LENGTH_MS, 0, PULSE_LENGTH_MS / 2, 0, 0);
    // redPWM.enableDeadbandElimination(true);
    // greenPWM.setBounds(PULSE_LENGTH_MS, 0, PULSE_LENGTH_MS / 2, 0, 0);
    // greenPWM.enableDeadbandElimination(true);
    // bluePWM.setBounds(PULSE_LENGTH_MS, 0, PULSE_LENGTH_MS / 2, 0, 0);
    // bluePWM.enableDeadbandElimination(true);

    // DigitalOutput redDio=new DigitalOutput(redPWM.getChannel());
    // redDO.updateDutyCycle();
    // SmartDashboard.putData("redPVM", redPWM);
    // SmartDashboard.putData("greenPVM", greenPWM);
    // SmartDashboard.putData("bluePVM", bluePWM);
    // }

    // public void setColor(Color color) {
    // redDO.updateDutyCycle(1.0 - (color.red / 255.0));
    // greenDO.updateDutyCycle(1.0 - (color.green / 255.0));
    // blueDO.updateDutyCycle(1.0 - (color.blue / 255.0));
    // System.out.println(
    // "Color has been set to " + color.red + ", " + color.green + ", " +
    // color.blue);
    // System.out.println(redPWM.getRaw() + ", " + greenPWM.getRaw() + ", "
    // + bluePWM.getRaw());
    // }
    @Override
    public void periodic() {
        clock = (clock + 1) & 255;
        setColor(colors[clock >> 6]);
        ledStrip.setData(buffer);
    }
}

// public static final Color whiteColor = new Color(255, 255, 255);
// public static final Color redColor = new Color(255, 0, 0);
// public static final Color greenColor = new Color(0, 255, 0);
// public static final Color blueColor = new Color(0, 0, 255);
// public static final Color yellowColor = new Color(255, 255, 0);
// public static final Color cyanColor = new Color(0, 255, 255);
// public static final Color magentaColor= new Color(255, 0, 255);
// public static final Color purpleColor = new Color(96, 0, 224);
// public static final Color off = new Color(0, 0, 0);