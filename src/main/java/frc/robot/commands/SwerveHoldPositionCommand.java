package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveHoldPositionCommand extends CommandBase {
    private final SwerveDrive swerve;

    public SwerveHoldPositionCommand(SwerveDrive swerve) {
        addRequirements(swerve);
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        swerve.holdPosition();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopDriving();
    }
}
