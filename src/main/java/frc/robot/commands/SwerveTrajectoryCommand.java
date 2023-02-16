package frc.robot.commands;

import static frc.robot.Constants.Auto.ANGULAR_KP;
import static frc.robot.Constants.Auto.ANGULAR_TOLERANCE;
import static frc.robot.Constants.Auto.LINEAR_KP;
import static frc.robot.Constants.Auto.LINEAR_TOLERANCE;
import static frc.robot.Constants.Auto.LOOKAHEAD_DISTANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.SwerveTrajectory;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveTrajectoryCommand extends CommandBase {
    private final SwerveDrive      swerveDrive;
    private final SwerveTrajectory trajectory;
    private final PIDController    xController;
    private final PIDController    yController;
    private final PIDController    rController;
    private final Pose2d           targetPose;
    private Pose2d                 currentPose;
    private int                    closestPointIndex;
    private int                    lookAheadPointIndex;

    public SwerveTrajectoryCommand(SwerveDrive swerveDrive,
            SwerveTrajectory trajectory) {
        this.swerveDrive = swerveDrive;
        this.trajectory = trajectory;

        xController = new PIDController(LINEAR_KP, 0, 0);
        yController = new PIDController(LINEAR_KP, 0, 0);
        rController = new PIDController(ANGULAR_KP, 0, 0);
        rController.enableContinuousInput(-Math.PI, Math.PI);

        targetPose = trajectory.points[trajectory.length - 1].pose;
        closestPointIndex = 0;
        lookAheadPointIndex = 0;
    }

    @Override
    public void execute() {
        updateCurrentPose();
        calculateGoals();
        driveToSetpoints();
    }

    private void updateCurrentPose() {
        currentPose = swerveDrive.getEstimatedPose();
    }

    private void findClosestPointIndex() {
        double minDistance = Double.MAX_VALUE;
        int index = -1;
        for (int i = closestPointIndex; i < trajectory.length; i++) {
            double distance = trajectory.points[i].pose.minus(currentPose)
                                                       .getTranslation()
                                                       .getNorm();
            if (distance < minDistance) {
                index = i;
                minDistance = distance;
            } else {
                break;
            }
        }
        if (index != -1) {
            closestPointIndex = index;
        }
    }

    private void findLookAheadPointIndex() {
        int index = -1;
        double maxDistance = 0;
        for (int i = lookAheadPointIndex; i < trajectory.length; i++) {
            double distance = trajectory.points[i].pose.minus(currentPose)
                                                       .getTranslation()
                                                       .getNorm();
            if (distance > maxDistance && distance < LOOKAHEAD_DISTANCE) {
                index = i;
                maxDistance = distance;
            } else {
                // Only want to get farther
                break;
            }
        }
        if (index == -1) {
            lookAheadPointIndex = closestPointIndex;
        } else {
            lookAheadPointIndex = index;
        }
    }

    private void calculateGoals() {
        findClosestPointIndex();
        findLookAheadPointIndex();
    }

    private void driveToSetpoints() {
        // Set to move towards lookahead pose
        Pose2d targetPose = trajectory.points[lookAheadPointIndex].pose;
        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        rController.setSetpoint(targetPose.getRotation()
                                          .getRadians());

        // Calculate velocities
        double vx = xController.calculate(currentPose.getX());
        double vy = yController.calculate(currentPose.getY());
        double vr = rController.calculate(currentPose.getRotation()
                                                     .getRadians());

        // Scale linear velocity to meet velocity constraints
        double velocity = Math.hypot(vx, vy);
        double commandedVelocity = Math.max(
                trajectory.points[closestPointIndex].commandedVelocity, 0.1);
        double ratio = commandedVelocity / velocity;
        vx *= ratio;
        vy *= ratio;

        swerveDrive.driveVelocity(vx, vy, vr);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stopDriving();
    }

    @Override
    public boolean isFinished() {
        if (closestPointIndex < trajectory.length - 3) {
            // Haven't run the whole route, start might be same as end
            // Allow for a bit of leeway (length-3 instead of length-1)
            return false;
        }
        Transform2d delta = targetPose.minus(swerveDrive.getEstimatedPose());
        return delta.getTranslation()
                    .getNorm() < LINEAR_TOLERANCE
                && Math.abs(delta.getRotation()
                                 .getDegrees()) < ANGULAR_TOLERANCE;
    }
}
