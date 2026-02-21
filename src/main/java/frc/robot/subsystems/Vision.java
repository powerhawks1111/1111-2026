// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
    private static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private static final Transform3d fieldToCamera = new Transform3d();
    private final PhotonPoseEstimator m_Estimator = new PhotonPoseEstimator(kTagLayout, fieldToCamera);

    public Vision() {
            
    }

    public void EstimatePose() {
        // var results = cam1.getAllUnreadResults();
        // for (var result : results) {
        //     var multitagResult = result.getMultiTagResult();
        //     if (multitagResult.isPresent()) {
        //         Transform3d fieldToCamera = multitagResult.get().estimatedPose.best;
        //     }
        // }

        var result = cam1.getLatestResult();
        if (result.hasTargets()) {
            var estimate = m_Estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = m_Estimator.estimateLowestAmbiguityPose(result);
            }
        }
        
    }
}