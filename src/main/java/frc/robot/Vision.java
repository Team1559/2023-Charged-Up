package frc.robot;

import static frc.robot.Constants.Vision.AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.Vision.CAMERA_NAME;
import static frc.robot.Constants.Vision.POSE_STRATEGY;
import static frc.robot.Constants.Vision.ROBOT_TO_CAMERA;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera             camera;
    private final AprilTagFieldLayout      aprilTagFieldLayout;
    private final PhotonPoseEstimator      photonPoseEstimator;
    private final SwerveDrivePoseEstimator swervePoseEstimator;
    private boolean                        poseSet = false;

    public Vision(SwerveDrivePoseEstimator poseEstimator) {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
        swervePoseEstimator = poseEstimator;
        camera = new PhotonCamera(CAMERA_NAME);

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, POSE_STRATEGY, camera,
                ROBOT_TO_CAMERA);
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        boolean resultPresent = result.isPresent();
        SmartDashboard.putBoolean("Apriltag", resultPresent);
        if (!resultPresent) {
            return;
        }
        boolean lowAmbiguity = camera.getLatestResult()
                                     .getBestTarget()
                                     .getPoseAmbiguity() < AMBIGUITY_THRESHOLD;
        if (!lowAmbiguity) {
            return;
        }

        EstimatedRobotPose pose = result.get();
        SmartDashboard.putNumber("Vision Pose X", pose.estimatedPose.toPose2d()
                                                                    .getX());
        SmartDashboard.putNumber("Vision Pose Y", pose.estimatedPose.toPose2d()
                                                                    .getY());
        SmartDashboard.putNumber("Vision Pose R", pose.estimatedPose.toPose2d()
                                                                    .getRotation()
                                                                    .getDegrees());
        if (Constants.FeatureFlags.CHASSIS_ENABLED) {
            try {
                double stdDev = 0.5 * camera.getLatestResult()
                                            .getBestTarget()
                                            .getBestCameraToTarget()
                                            .getTranslation()
                                            .toTranslation2d()
                                            .getNorm();
                SmartDashboard.putNumber("Vision StdDev", stdDev);
                if (poseSet) {
                    swervePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                            pose.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
                } else {
                    swervePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                            pose.timestampSeconds, VecBuilder.fill(0, 0, 0));
                    swervePoseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                            pose.timestampSeconds, VecBuilder.fill(stdDev, stdDev, stdDev));
                    poseSet = true;
                }
            } catch (ConcurrentModificationException e) {
                SmartDashboard.putString("Error", e.toString());
            }
        }
    }
}
