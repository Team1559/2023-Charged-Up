package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveDriveRotate180Command extends CommandBase {
    private final SwerveDrive swerve;
    private final boolean     turnRight;

    private Rotation2d target;

    public SwerveDriveRotate180Command(SwerveDrive swerve, boolean turnRight) {
        this.swerve = swerve;
        this.turnRight = turnRight;
    }

    @Override
    public void initialize() {
        target = normalize(swerve.getEstimatedPose()
                                 .getRotation()
                                 .unaryMinus());

    }

    @Override
    public void execute() {
        Rotation2d setpoint = swerve.getEstimatedPose()
                                    .getRotation()
                                    .plus(Rotation2d.fromDegrees(turnRight ? -90 : 90));
        double diff = target.getDegrees() - setpoint.getDegrees();
        if (turnRight && diff < 0 || !turnRight && diff > 0) {
            setpoint = target;
        }
        swerve.setRSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return swerve.getEstimatedPose()
                     .getRotation()
                     .minus(target)
                     .getDegrees() < 5;
    }

    private static Rotation2d normalize(Rotation2d r) {
        double normal = r.plus(Rotation2d.fromDegrees(0))
                         .getDegrees();
        normal = 180 * Math.round(normal / 180);
        return Rotation2d.fromDegrees(normal);
    }
}
