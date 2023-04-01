package frc.robot;

import static frc.robot.Constants.Vision.AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.Vision.CAMERA_NAME_BACK;
import static frc.robot.Constants.Vision.CAMERA_NAME_FRONT;
import static frc.robot.Constants.Vision.POSE_STRATEGY;
import static frc.robot.Constants.Vision.ROBOT_TO_CAMERA_BACK;
import static frc.robot.Constants.Vision.ROBOT_TO_CAMERA_FRONT;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera             cameraFront;
    private final PhotonCamera             cameraBack;
    private final AprilTagFieldLayout      aprilTagFieldLayout;
    private final PhotonPoseEstimator      poseEstimatorFront;
    private final PhotonPoseEstimator      poseEstimatorBack;
    private final SwerveDrivePoseEstimator swervePoseEstimator;
    private boolean                        poseSet;

    public Vision(SwerveDrivePoseEstimator poseEstimator) {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
        swervePoseEstimator = poseEstimator;
        cameraFront = new PhotonCamera(CAMERA_NAME_FRONT);
        cameraBack = new PhotonCamera(CAMERA_NAME_BACK);
        poseEstimatorFront = new PhotonPoseEstimator(aprilTagFieldLayout, POSE_STRATEGY,
                cameraFront, ROBOT_TO_CAMERA_FRONT);
        poseEstimatorBack = new PhotonPoseEstimator(aprilTagFieldLayout, POSE_STRATEGY, cameraBack,
                ROBOT_TO_CAMERA_BACK);
    }

    @Override
    public void periodic() {
        front: do {
            Optional<EstimatedRobotPose> estimatedPoseFront = poseEstimatorFront.update();
            PhotonPipelineResult cameraResult = cameraFront.getLatestResult();
            boolean posePresent = estimatedPoseFront.isPresent() && cameraResult.hasTargets()
                    && cameraResult.getBestTarget() != null;
            if (!posePresent) {
                break front;
            }
            SmartDashboard.putBoolean("Apriltag", posePresent);
            boolean lowAmbiguity = cameraResult.getBestTarget()
                                               .getPoseAmbiguity() < AMBIGUITY_THRESHOLD;
            if (!lowAmbiguity) {
                break front;
            }

            EstimatedRobotPose pose = estimatedPoseFront.get();
            SmartDashboard.putNumber("Vision Pose X", pose.estimatedPose.toPose2d()
                                                                        .getX());
            SmartDashboard.putNumber("Vision Pose Y", pose.estimatedPose.toPose2d()
                                                                        .getY());
            SmartDashboard.putNumber("Vision Pose R", pose.estimatedPose.toPose2d()
                                                                        .getRotation()
                                                                        .getDegrees());
            if (Constants.FeatureFlags.CHASSIS_ENABLED) {
                try {
                    double stdDev = cameraResult.getBestTarget()
                                                .getBestCameraToTarget()
                                                .getTranslation()
                                                .toTranslation2d()
                                                .getNorm();
                    if (stdDev > 5) {
                        break front;
                    }
                    if (poseSet) {
                        swervePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                pose.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
                    } else {
                        setInitialPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                    }
                } catch (ConcurrentModificationException e) {
                    // ignore
                }
            }
            return;
        } while (false);
        back: do {
            Optional<EstimatedRobotPose> estimatedPoseBack = poseEstimatorBack.update();
            PhotonPipelineResult cameraResult = cameraBack.getLatestResult();
            boolean posePresent = estimatedPoseBack.isPresent() && cameraResult.hasTargets()
                    && cameraResult.getBestTarget() != null;
            if (!posePresent) {
                break back;
            }
            SmartDashboard.putBoolean("Apriltag", posePresent);
            boolean lowAmbiguity = cameraResult.getBestTarget()
                                               .getPoseAmbiguity() < AMBIGUITY_THRESHOLD;
            if (!lowAmbiguity) {
                break back;
            }

            EstimatedRobotPose pose = estimatedPoseBack.get();
            SmartDashboard.putNumber("Vision Pose X", pose.estimatedPose.toPose2d()
                                                                        .getX());
            SmartDashboard.putNumber("Vision Pose Y", pose.estimatedPose.toPose2d()
                                                                        .getY());
            SmartDashboard.putNumber("Vision Pose R", pose.estimatedPose.toPose2d()
                                                                        .getRotation()
                                                                        .getDegrees());
            if (Constants.FeatureFlags.CHASSIS_ENABLED) {
                try {
                    double stdDev = cameraResult.getBestTarget()
                                                .getBestCameraToTarget()
                                                .getTranslation()
                                                .toTranslation2d()
                                                .getNorm();
                    if (stdDev > 5) {
                        break back;
                    }
                    if (poseSet) {
                        swervePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                pose.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
                    } else {
                        setInitialPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
                    }
                } catch (ConcurrentModificationException e) {
                    // ignore
                }
            }
        } while (false);
        SmartDashboard.putBoolean("Apriltag", false);

    }

    public void setInitialPose(Pose2d pose, double timestampSeconds) {
        swervePoseEstimator.addVisionMeasurement(pose, timestampSeconds, VecBuilder.fill(0, 0, 0));
        poseSet = true;
    }

    public boolean isPoseSet() {
        return poseSet;
    }
}
