package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;

    /*Constructor method for Vision class */
    public Vision() {
        camera = new PhotonCamera("camera");
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }
    
    
    @Override
    public void periodic(){
        // Query the latest result from PhotonVision
        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putBoolean("Has Target", result.hasTargets());
        
        if (!result.hasTargets()){
            return;
        }
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        SmartDashboard.putNumber("Target ID", bestTarget.getFiducialId());
    }
}
