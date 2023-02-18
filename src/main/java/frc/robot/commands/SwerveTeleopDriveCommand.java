package frc.robot.commands;

import static frc.robot.Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DTXboxController;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveTeleopDriveCommand extends CommandBase {
    private final DTXboxController controller;
    private final SwerveDrive      swerve;

    public SwerveTeleopDriveCommand(SwerveDrive swerve,
            DTXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        controller.setDeadBand(0.075);
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        double vx = controller.getLeftStickYSquared() * MAXIMUM_LINEAR_VELOCITY;
        double vy = -controller.getLeftStickXSquared()
                * MAXIMUM_LINEAR_VELOCITY;
        double vr = -controller.getRightStickXSquared()
                * MAXIMUM_ANGULAR_VELOCITY;
        if (DriverStation.getAlliance() == Alliance.Red) {
            // Invert control from red perspective to keep it consistent
            vx = -vx;
            vy = -vy;
        }
        swerve.driveVelocity(vx, vy, vr);
    }
}
