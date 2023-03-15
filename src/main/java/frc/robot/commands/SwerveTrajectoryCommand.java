package frc.robot.commands;

import static frc.robot.Constants.Auto.ANGULAR_TOLERANCE;
import static frc.robot.Constants.Auto.LINEAR_TOLERANCE;
import static frc.robot.Constants.Auto.LOOKAHEAD_DISTANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.lib.SwerveTrajectory;

import frc.robot.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveTrajectoryCommand extends CommandBase {
    private final SwerveDrive      swerveDrive;
    private final SwerveTrajectory trajectory;
    private final Vision           vision;
    private final Pose2d           targetPose;
    private Pose2d                 currentPose;
    private int                    closestPointIndex;
    private int                    lookAheadPointIndex;

    public SwerveTrajectoryCommand(SwerveDrive swerveDrive, SwerveTrajectory trajectory,
            Vision vision) {
        this.swerveDrive = swerveDrive;
        this.trajectory = trajectory;
        this.vision = vision;

        targetPose = trajectory.points[trajectory.length - 1].pose;
        closestPointIndex = 0;
        lookAheadPointIndex = 0;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        if (swerveDrive.getEstimatedPose()
                       .minus(trajectory.points[0].pose)
                       .getTranslation()
                       .getNorm() > 1) {
            cancel();
        }
        swerveDrive.setFieldRelative();
    }

    @Override
    public void execute() {
        if (!vision.isPoseSet()) {
            rotateSlowly();
            return;
        }

        currentPose = swerveDrive.getEstimatedPose();
        calculateGoals();
        driveToSetpoints();
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
        // Translation2d robotToTarget = targetPose.minus(currentPose)
        // .getTranslation();
        Translation2d robotToTarget = targetPose.getTranslation()
                                                .minus(currentPose.getTranslation());
        // Convert from linear displacement to linear velocity
        double scalar = trajectory.points[closestPointIndex].commandedVelocity
                / robotToTarget.getNorm();
        robotToTarget = robotToTarget.times(scalar);

        // Calculate velocities
        double vx = robotToTarget.getX();
        double vy = robotToTarget.getY();

        swerveDrive.setRSetpoint(targetPose.getRotation());
        swerveDrive.driveVelocity(vx, vy, 0);
    }

    private void rotateSlowly() {
        swerveDrive.driveVelocity(0, 0, 0.5);
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
