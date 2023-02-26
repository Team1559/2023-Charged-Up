package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lighting {
    private DigitalOutput redDO;
    private DigitalOutput greenDO;
    private DigitalOutput blueDO;
    private final double  PULSE_LENGTH_MS = 5.005;

    public Lighting(int pwmRed, int pwmGreen, int pwmBlue) {
        // redPWM = new PWM(pwmRed);
        // greenPWM = new PWM(pwmGreen);
        // bluePWM = new PWM(pwmBlue);
        redDO = new DigitalOutput(pwmRed);
        redDO.enablePWM(0);
        greenDO = new DigitalOutput(pwmGreen);
        greenDO.enablePWM(0);
        blueDO = new DigitalOutput(pwmBlue);
        blueDO.enablePWM(0);

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
    }

    public void setColor(Color color) {
        redDO.updateDutyCycle(1.0 - (color.red / 255.0));
        greenDO.updateDutyCycle(1.0 - (color.green / 255.0));
        blueDO.updateDutyCycle(1.0 - (color.blue / 255.0));
        // System.out.println(
        // "Color has been set to " + color.red + ", " + color.green + ", " +
        // color.blue);
        // System.out.println(redPWM.getRaw() + ", " + greenPWM.getRaw() + ", "
        // + bluePWM.getRaw());
    }

}
