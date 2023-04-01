package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveDrivePidBalanceCommand extends CommandBase {
    private final SwerveDrive swerve;
    private final double      kP;
    private final double      kI;
    private final double      kD;
    private final int         cycleThreshold;
    private double            integralAccumulator;
    private int               triggerCycleCount;

    public SwerveDrivePidBalanceCommand(SwerveDrive swerve, double kP, double kI, double kD,
            int cycleThreshold) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.cycleThreshold = cycleThreshold;
        integralAccumulator = 0;
        triggerCycleCount = 0;
    }

    @Override
    public void initialize() {
        swerve.setRobotRelative();
    }

    @Override
    public void execute() {
        double pitch = swerve.getGyroPitchDegrees();
        double pitchVelocity = swerve.getGyroPitchVelocity();

        if (Math.abs(pitch) < 3 && Math.abs(pitchVelocity) < 2) {
            triggerCycleCount++;
        } else {
            triggerCycleCount = 0;
        }

        if (Math.abs(pitch) < 5) {
            integralAccumulator += pitch;
        } else {
            integralAccumulator = 0;
        }
        double outputVelocity = pitch * kP + integralAccumulator * kI + pitchVelocity * kD;
        swerve.driveVelocity(outputVelocity, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopDriving();
    }

    @Override
    public boolean isFinished() {
        return triggerCycleCount >= cycleThreshold;
    }
}
