package frc.robot.commands;

import static frc.robot.Constants.Swerve.MAXIMUM_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAXIMUM_LINEAR_VELOCITY;
import static frc.robot.Constants.Swerve.SLOW_MODE_RATIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.lib.DTXboxController;

import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveTeleopAlignToGridCommand extends CommandBase {
    // 0.559 meters apart each (approx.)
    private static final double[] GRID_POSITIONS = { 0.513, 1.072, 1.631, 2.189, 2.748, 3.307,
            3.865, 4.424, 4.983 };

    private final DTXboxController controller;
    private final SwerveDrive      swerve;
    private final PIDController    pid;

    // Needs the controller to still accept X and R input during the command
    public SwerveTeleopAlignToGridCommand(SwerveDrive swerve, DTXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        this.pid = new PIDController(2, 0, 0.5);
        controller.setDeadBand(0.075);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setRobotRelative();

        // Find closest grid node Y
        double currentY = swerve.getEstimatedPose()
                                .getY();
        double bestError = 2;
        int bestIndex = -1;
        for (int i = 0; i < GRID_POSITIONS.length; i++) {
            double error = Math.abs(currentY - GRID_POSITIONS[i]);
            if (error < bestError) {
                bestError = error;
                bestIndex = i;
            }
        }
        if (bestIndex == -1) {
            // Error: robot thinks it is too far from any grid node Y
            cancel();
        }
        pid.setSetpoint(GRID_POSITIONS[bestIndex]);
    }

    @Override
    public void execute() {
        // If significant Y velocity is requested, cancel
        if (Math.abs(controller.getLeftStickYSquared()) > 0.75) {
            swerve.stopDriving();
            cancel();
            return;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        // Accept X and R input as normal (R will override PID if requested)
        double vx = controller.getLeftStickYSquared() * MAXIMUM_LINEAR_VELOCITY;
        double vr = controller.getRightStickXSquared() * MAXIMUM_ANGULAR_VELOCITY;
        if (DriverStation.getAlliance() == Alliance.Red) {
            // Invert control from red perspective to keep it consistent
            vx = -vx;
        }
        if (controller.getLeftTrigger() >= 0.5) {
            vx *= SLOW_MODE_RATIO;
            vr *= SLOW_MODE_RATIO;
        }

        // Calculate Y velocity, and add driver X input
        double vy = pid.calculate(swerve.getEstimatedPose()
                                        .getY());
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, vy, vr,
                swerve.getRobotAngle());
        speeds.vxMetersPerSecond += vx;

        swerve.setRSetpoint(Rotation2d.fromDegrees(
                DriverStation.getAlliance() == DriverStation.Alliance.Red ? 180 : 0));
        swerve.driveVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopDriving();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
