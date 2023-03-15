package frc.robot.commands;

import static frc.robot.Constants.Swerve.ROTATION_SNAP_THRESHOLD;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveTeleopSnapRotateCommand extends CommandBase {
    private final SwerveDrive swerve;
    private final boolean     turnRight;

    public SwerveTeleopSnapRotateCommand(SwerveDrive swerve, boolean turnRight) {
        this.swerve = swerve;
        this.turnRight = turnRight;
    }

    @Override
    public void initialize() {
        double setpoint = swerve.getRSetpoint();
        double currentPosition;
        if (Double.isNaN(setpoint)) {
            currentPosition = swerve.getRobotAngle()
                                    .getDegrees();
        } else {
            currentPosition = Math.toDegrees(setpoint);
        }

        double closest45 = 45 + 90D * Math.round((currentPosition - 45) / 90);
        double closest90 = turnRight ? (closest45 - 45) : (closest45 + 45);
        double setpointDeg = closest90;

        double diff = Math.abs(closest90 - currentPosition);
        if (diff <= ROTATION_SNAP_THRESHOLD) {
            setpointDeg += turnRight ? -90 : 90;
        }

        swerve.setRSetpoint(Rotation2d.fromDegrees(setpointDeg));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
