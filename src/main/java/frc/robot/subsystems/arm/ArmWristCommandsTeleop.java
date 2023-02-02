package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.DTXboxController;

public class ArmWristCommandsTeleop extends CommandBase {

    private DTXboxController controller;
    private ArmWrist wrist;

    private final double deadband          = 0.05;
    private final double angleVelPerSecond = 90D;
    private final double angleVelPerCycle  = angleVelPerSecond / 50.0;

    public ArmWristCommandsTeleop(ArmWrist armWrist, DTXboxController controller) {
        this.wrist = armWrist;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double x = controller.getLeftStickX();
        double angle = wrist.getWrist();
        if (Math.abs(x) > deadband) {
            if (x < 0) {
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
        wrist.setAngle(angle);
    }
}