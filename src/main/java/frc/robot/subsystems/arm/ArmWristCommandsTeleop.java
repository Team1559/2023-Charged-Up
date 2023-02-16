package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DTXboxController;

public class ArmWristCommandsTeleop extends CommandBase {

    private DTXboxController controller;
    private ArmWrist         wrist;

    private final double deadband          = 0.05;
    private final double angleVelPerSecond = 90D;
    private final double angleVelPerCycle  = angleVelPerSecond / 50.0;

    public ArmWristCommandsTeleop(ArmWrist armWrist,
            DTXboxController controller) {
        this.wrist = armWrist;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double rightStickY = controller.getRightStickY();
        double angle = wrist.getAngle();
        if (Math.abs(rightStickY) > deadband) {
            if (rightStickY < 0) {
                angle -= angleVelPerCycle;
            } else {
                angle += angleVelPerCycle;
            }
        } else {

        }
        if (Math.abs(angle) > 90) {
            angle = 90;
        } else if (angle < -90) {
            angle = -90;
        }
        wrist.setDestinationAngle(angle);
    }
}