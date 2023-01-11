// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;

    private Field2d m_field = new Field2d();

    private static final String CAMERA_NAME = "Global_Shutter_Camera";

    private int updateCycles = 0;

    /** Creates a new Vision. */
    public Vision() {
        m_camera = new PhotonCamera(CAMERA_NAME);

        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException();
        }
    }

    public Pose3d getLatestPose() {
        PhotonPipelineResult result = m_camera.getLatestResult();
        
        boolean hasTarget = result.hasTargets();

        if (hasTarget) {
            PhotonTrackedTarget target = result.getBestTarget();

            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose;
            }
        }
        return new Pose3d();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateCycles++;
        if (updateCycles == 1) {
            m_field.setRobotPose(getLatestPose().toPose2d());
            updateCycles = 0;
        }
        SmartDashboard.putData(m_field);
    }
}
