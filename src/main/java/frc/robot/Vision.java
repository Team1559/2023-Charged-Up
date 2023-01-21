package frc.robot;

import java.io.IOException;
import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private PhotonCamera        camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private Pose3d lastPose;

    /* Constructor method for Vision class */
    public Vision() {
        camera = new PhotonCamera("OV5647");
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
        double cameraXOffset = Units.inchesToMeters(16);
        double cameraZOffset = Units.inchesToMeters(5.5);
        Transform3d robotToCam = new Transform3d(
                new Translation3d(cameraXOffset, 0.0, cameraZOffset),
                new Rotation3d(0, 0, 0));
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
        // lastPose = new Pose3d(new Translation3d(), new Rotation3d());
    }

    @Override
    public void periodic() {
        // photonPoseEstimator.setReferencePose(lastPose);
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if (result.isPresent()) {
            EstimatedRobotPose pose = result.get();
            lastPose = new Pose3d(pose.estimatedPose.getTranslation(), pose.estimatedPose.getRotation());
            SmartDashboard.putNumber("pose X", lastPose.getX());
            SmartDashboard.putNumber("pose Y", lastPose.getY());
            SmartDashboard.putNumber("pose Z", lastPose.getZ());
            SmartDashboard.putNumber("pose Heading", lastPose.getRotation()
                                                           .getAngle() / Math.PI * 180);
            SmartDashboard.putNumber("pose Timestamp", pose.timestampSeconds);
            // new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(),
            // currentTime - result.get().getSecond());
        }
        SmartDashboard.putString("Result str", Objects.toString(result));
        // Query the latest result from PhotonVision
        // PhotonPipelineResult result = camera.getLatestResult();
        // SmartDashboard.putBoolean("Has Target", result.hasTargets());
        // SmartDashboard.putNumber("Result Timestamp",
        // result.getTimestampSeconds());
        // SmartDashboard.putNumber("Time", System.currentTimeMillis());

        // if (!result.hasTargets()){
        // return;
        // }
        // PhotonTrackedTarget bestTarget = result.getBestTarget();
        // SmartDashboard.putNumber("Target ID", bestTarget.getFiducialId());
    }
}
