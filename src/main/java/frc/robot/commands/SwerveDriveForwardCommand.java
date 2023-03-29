package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveDriveForwardCommand extends CommandBase {
    private final SwerveDrive     swerve;
    private final double          velocity;
    private final BooleanSupplier isFinished;

    public SwerveDriveForwardCommand(SwerveDrive swerve, double velocity,
            BooleanSupplier isFinished) {
        this.swerve = swerve;
        this.velocity = velocity;
        this.isFinished = isFinished;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setRobotRelative();
    }

    @Override
    public void execute() {
        swerve.driveVelocity(velocity, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
