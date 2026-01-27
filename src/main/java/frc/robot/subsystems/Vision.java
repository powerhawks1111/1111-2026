// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera cam1 = new PhotonCamera(CameraConstants.pvCamOne);
    private final PhotonCamera cam2 = new PhotonCamera(CameraConstants.pvCamTwo);
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static Transform3d fieldToCamera = new Transform3d();
        public Vision() {
            
        }
    
        public boolean tagInSight() {
            return (cam1.getLatestResult().hasTargets() || cam2.getLatestResult().hasTargets());
        }
    
        //temp method until i know what im doing. important thing is that it gives a reference point of where i get data from
        public List<PhotonPipelineResult> getLatestResult() {
            if (cam1.getLatestResult().hasTargets()) { //if we have targets
                return cam1.getAllUnreadResults();
            } else { //we are assuming that the user has already called tagInSight above, so if #1 has none we resort to this. 
                return cam2.getAllUnreadResults();
            } 
        }
    
        public Pose2d getPoseMultiTag() {
            List<PhotonPipelineResult> results = cam1.getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                var multiTagResult = result.getMultiTagResult();
                if (multiTagResult.isPresent()) {
                    fieldToCamera = multiTagResult.get().estimatedPose.best;
            }
        }
        return new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(), Rotation2d.fromDegrees(cam1.getLatestResult().getBestTarget().getYaw()));
    }
}