package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lighting {
    private PWM          redPWM;
    private PWM          greenPWM;
    private PWM          bluePWM;
    private final double PULSE_LENGTH_MS = 5.005;

    public Lighting(int pwmRed, int pwmGreen, int pwmBlue) {
        redPWM = new PWM(pwmRed);
        greenPWM = new PWM(pwmGreen);
        bluePWM = new PWM(pwmBlue);

        redPWM.setBounds(PULSE_LENGTH_MS, 0, PULSE_LENGTH_MS / 2, 0, 0);
        redPWM.enableDeadbandElimination(true);
        greenPWM.setBounds(PULSE_LENGTH_MS, 0, PULSE_LENGTH_MS / 2, 0, 0);
        greenPWM.enableDeadbandElimination(true);
        bluePWM.setBounds(PULSE_LENGTH_MS, 0, PULSE_LENGTH_MS / 2, 0, 0);
        bluePWM.enableDeadbandElimination(true);

        // DigitalOutput redDio=new DigitalOutput(redPWM.getChannel());
        // redDio.updateDutyCycle(1);
        SmartDashboard.putData("redPVM", redPWM);
        SmartDashboard.putData("greenPVM", greenPWM);
        SmartDashboard.putData("bluePVM", bluePWM);
    }

    public void setColor(Color color) {
        redPWM.setRaw(color.red);
        greenPWM.setRaw(color.green);
        bluePWM.setRaw(color.blue);
        System.out.println(
                "Color has been set to " + color.red + ", " + color.green + ", " + color.blue);
        System.out.println(redPWM.getRaw() + ", " + greenPWM.getRaw() + ", " + bluePWM.getRaw());
    }

}
