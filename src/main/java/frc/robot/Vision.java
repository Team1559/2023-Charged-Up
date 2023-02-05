package frc.robot;

import static frc.robot.Constants.Vision.CAMERA_NAME;
import static frc.robot.Constants.Vision.POSE_STRATEGY;
import static frc.robot.Constants.Vision.ROBOT_TO_CAMERA;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera                   camera;
    private final AprilTagFieldLayout      aprilTagFieldLayout;
    private final PhotonPoseEstimator      photonPoseEstimator;
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    public Vision(SwerveDrivePoseEstimator poseEstimator) {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
        swervePoseEstimator = poseEstimator;
        camera = new PhotonCamera(CAMERA_NAME);

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                POSE_STRATEGY, camera, ROBOT_TO_CAMERA);
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        boolean resultPresent = result.isPresent();
        PhotonTrackedTarget bestTarget = camera.getLatestResult()
                                               .getBestTarget();
        boolean lowAmbiguity = bestTarget != null
                && bestTarget.getPoseAmbiguity() < 0.2;
        SmartDashboard.putBoolean("Apriltag", resultPresent);

        if (resultPresent && lowAmbiguity) {
            EstimatedRobotPose pose = result.get();
            SmartDashboard.putNumber("Vision Pose X",
                    pose.estimatedPose.toPose2d()
                                      .getX());
            SmartDashboard.putNumber("Vision Pose Y",
                    pose.estimatedPose.toPose2d()
                                      .getY());
            SmartDashboard.putNumber("Vision Pose R",
                    pose.estimatedPose.toPose2d()
                                      .getRotation()
                                      .getDegrees());
            if (Constants.FeatureFlags.CHASSIS_ENABLED) {
                try {
                    swervePoseEstimator.addVisionMeasurement(
                            pose.estimatedPose.toPose2d(),
                            pose.timestampSeconds);
                } catch (ConcurrentModificationException e) {
                    // ignore
                }
            }
        }
    }
}
