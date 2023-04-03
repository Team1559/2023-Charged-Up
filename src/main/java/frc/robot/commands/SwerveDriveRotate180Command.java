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
        target = snap(swerve.getEstimatedPose()
                                 .getRotation()
                                 .getDegrees()
                + 180);
    }

    @Override
    public void execute() {
        // Rotation2d setpoint = swerve.getEstimatedPose()
        //                             .getRotation()
        //                             .plus(Rotation2d.fromDegrees(turnRight ? -90 : 90));
        // double diff = target.getDegrees() - setpoint.getDegrees();
        // if ((turnRight ^ (diff > 0))) {
        //     setpoint = target;
        // }
        swerve.setRSetpoint(target);
        swerve.driveVelocity(0, 0, 0); // need to drive to activate PID
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getEstimatedPose()
                              .getRotation()
                              .minus(target)
                              .getDegrees()) <= 5;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopDriving();
    }

    private static Rotation2d snap(double r) {
        double normal = 180D * Math.round(r / 180D);
        return Rotation2d.fromDegrees(normal % 360);
    }
}
