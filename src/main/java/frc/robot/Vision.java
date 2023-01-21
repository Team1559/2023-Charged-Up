package frc.robot;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera             camera;
    private AprilTagFieldLayout      aprilTagFieldLayout;
    private PhotonPoseEstimator      photonPoseEstimator;
    private SwerveDrivePoseEstimator swervePoseEstimator;

    /* Constructor method for Vision class */
    public Vision(SwerveDrivePoseEstimator swervePoseEstimator) {
        this.swervePoseEstimator = swervePoseEstimator;
        camera = new PhotonCamera("OV5647");
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            throw new IllegalStateException(e);
        }
        double cameraXOffset = Units.inchesToMeters(16);
        double cameraZOffset = Units.inchesToMeters(5.5);
        Transform3d robotToCam = new Transform3d(
                new Translation3d(0,
                        cameraXOffset, cameraZOffset),
                new Rotation3d(0, 0, Math.toRadians(90)));
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if (result.isPresent()) {
            EstimatedRobotPose pose = result.get();
            long start = System.nanoTime();
            try {
                swervePoseEstimator.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            } catch (ConcurrentModificationException e) {
                SmartDashboard.putString("Exception", e.getMessage());
            }
            double time = (System.nanoTime() - start) * 1e-9;
            SmartDashboard.putNumber("Backcalculate time", time);
            SmartDashboard.putBoolean("Backcalculate", time < 0.005);
            SmartDashboard.putBoolean("Apriltag", true);
        } else {
            SmartDashboard.putBoolean("Apriltag", false);
        }
    }
}
