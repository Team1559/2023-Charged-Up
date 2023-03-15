package frc.robot.commands;

import static frc.robot.Constants.Swerve.HIGH_MODE_RATIO;
import static frc.robot.Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;
import static frc.robot.Constants.Swerve.MED_MODE_RATIO;
import static frc.robot.Constants.Swerve.SLOW_MODE_RATIO;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.lib.DTXboxController;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveTeleopDriveCommand extends CommandBase {
    private final DTXboxController controller;
    private final SwerveDrive      swerve;

    public SwerveTeleopDriveCommand(SwerveDrive swerve, DTXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        controller.setDeadBand(0.075);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setRobotRelative();
    }

    @Override
    public void execute() {
        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        double vx = controller.getLeftStickYSquared() * MAXIMUM_LINEAR_VELOCITY;
        double vy = -controller.getLeftStickXSquared() * MAXIMUM_LINEAR_VELOCITY;
        double vr = -controller.getRightStickXSquared() * MAXIMUM_ANGULAR_VELOCITY;

        if (DriverStation.getAlliance() == Alliance.Red && swerve.isFieldRelative()) {
            // Invert control from red perspective to keep it consistent
            vx = -vx;
            vy = -vy;
        }

        if (controller.getLeftTrigger() >= 0.5) {
            vx *= SLOW_MODE_RATIO;
            vy *= SLOW_MODE_RATIO;
            vr *= SLOW_MODE_RATIO;
        } else if (controller.getRightTrigger() >= 0.5) {
            vx *= MED_MODE_RATIO;
            vy *= MED_MODE_RATIO;
            vr *= MED_MODE_RATIO;
        } else if (controller.getRightBumper()) {
            vx *= HIGH_MODE_RATIO;
            vy *= HIGH_MODE_RATIO;
            vr *= HIGH_MODE_RATIO;
        }
        swerve.driveVelocity(vx, vy, vr);
    }
}
