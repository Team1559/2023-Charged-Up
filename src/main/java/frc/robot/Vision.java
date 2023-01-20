package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class Vision extends SubsystemBase {

    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    /*Constructor method for Vision class */
    public Vision() {
        camera = new PhotonCamera("OV5647");
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        double cameraXOffset = Units.inchesToMeters(16);
        double cameraZOffset = Units.inchesToMeters(5.5);
        Transform3d robotToCam = new Transform3d(new Translation3d(cameraXOffset, 0.0, cameraZOffset), new Rotation3d(0,0,0)); 
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);

    }
    
    
    @Override
    public void periodic(){
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        if (result.isPresent()) {
            EstimatedRobotPose pose = result.get();
            Pose2d pose2d = pose.estimatedPose.toPose2d();
            SmartDashboard.putNumber("pose X", pose2d.getX());
            SmartDashboard.putNumber("pose Y", pose2d.getY());
            SmartDashboard.putNumber("pose Heading", pose2d.getRotation().getDegrees());
            SmartDashboard.putNumber("pose Timestamp", pose.timestampSeconds);
             //new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        }
        SmartDashboard.putString("Result str", Objects.toString(result));
        // Query the latest result from PhotonVision
        // PhotonPipelineResult result = camera.getLatestResult();
        // SmartDashboard.putBoolean("Has Target", result.hasTargets());
        // SmartDashboard.putNumber("Result Timestamp", result.getTimestampSeconds());
        // SmartDashboard.putNumber("Time", System.currentTimeMillis());
        
        // if (!result.hasTargets()){
        //     return;
        // }
        // PhotonTrackedTarget bestTarget = result.getBestTarget();
        // SmartDashboard.putNumber("Target ID", bestTarget.getFiducialId());
    }
}
