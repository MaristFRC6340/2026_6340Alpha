// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.opencv.dnn.Net;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.Vision;

public class VisionSubsystem extends SubsystemBase {

  // Create Camera and Bind to Network Table
  PhotonCamera camera = new PhotonCamera("turretcamera");

  private double turnError;
  private double kp = 0.02;
  private double turnPower;

  int [] targetIDs = {8, 9, 10, 11, 12, 13}; //temp tags idk what they are

  // Access Network Table
  private NetworkTable photonTable;
  private NetworkTableEntry targetYaw;
  private NetworkTableEntry targetPitch;
  private NetworkTableEntry pipeline;

  // for mapping distance to hood angle
  public final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  // for turret auto-aim
  private double finalYaw;
  

  /** Creates a new Vision. */
  public VisionSubsystem() {
    // Constructor - not used yet

    // Establish SmartDashboard Value
    SmartDashboard.putNumber("Yaw", 0);
    // SmartDashboard.putNumber("Test Yaw", 0);
    // SmartDashboard.putNumber("Test Pitch", 0);
    SmartDashboard.putNumber("PipelineIndex", -1);
    // Bind to Network Table
    photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    targetYaw = photonTable.getEntry("turretcamera/targetYaw");
    targetPitch = photonTable.getEntry("turretcamera/targetPitch");
    pipeline = photonTable.getEntry("turretcamera/pipelineIndexState");
    
    if (pipeline.getDouble(-1) < 1) {
       pipeline.setInteger(1);
    }
    else {
      pipeline.setInteger(0);
    }

    // # SHOOTER LOOKUP
    SmartDashboard.putNumber("Robot Distance (2D)", 0);
    SmartDashboard.putNumber("Estimated Angle", 0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Read Data from Camera
    // Maybe put on Advantage Scope
    PhotonPipelineResult result = camera.getLatestResult();

    int pipelineIndex = (int)pipeline.getInteger(-1);
    SmartDashboard.putNumber("PipelineIndex", pipelineIndex);

    if (result.hasTargets()) {

      // Get All the Targets
      List<PhotonTrackedTarget> targets = result.getTargets();

      // Get Best Target
      PhotonTrackedTarget target = getBestTag(targets);



      if (target != null) {
        // Get Yaw
        Transform3d targetPose = target.getBestCameraToTarget(); // translation + rotation
        Translation3d translation = targetPose.getTranslation(); // Rotation3d
        double finalYawRad = Math.atan2(translation.getY(), translation.getX());
        finalYaw = Math.toDegrees(finalYawRad);
        SmartDashboard.putNumber("Final Yaw", finalYaw);
        // System.out.println("Transform3d: " + targetPose);
        // double testYaw = target.getYaw();
        // SmartDashboard.putNumber("Test Yaw", testYaw);
        // Get Pitch
        // double testPitch = target.getPitch();
        // SmartDashboard.putNumber("Test Pitch", testPitch);
        // Get Distance
        double dist = getDistance(target);
        SmartDashboard.putNumber("Robot Distance (2D)", dist);
      }
      // System.out.println(target.getFiducialId());
    }
    double yaw = targetYaw.getDouble(0);
    SmartDashboard.putNumber("Yaw", yaw);
    double pitch = targetPitch.getDouble(0);
    SmartDashboard.putNumber("Pitch", pitch);
    //SmartDashboard.putNumber("optimal ID", target.getFiducialId());

}

  public PhotonTrackedTarget getBestTag(List<PhotonTrackedTarget> targetsSeen)
  {
    int bestID = -1;
    PhotonTrackedTarget optimalTarget = null;
    if(targetsSeen != null)
    {
      double bestDistance = Double.MAX_VALUE;
      for (PhotonTrackedTarget t : targetsSeen) 
      {
        for (int targetID : targetIDs)
        {
            if (t.getFiducialId() != targetID) continue;
            double distance = Math.abs(t.getBestCameraToTarget().getX());

            if(distance < bestDistance) 
            {
                bestID = t.getFiducialId();
                bestDistance = distance;
                optimalTarget = t;
            }
        }
      }
    }
    //if some error with getting targets return -1
    return optimalTarget;
  }

  public double getFinalYaw() {
    return finalYaw;
  }

  public double getDistance(PhotonTrackedTarget target) {
        Transform3d camToTarget = target.getBestCameraToTarget();

        double x = camToTarget.getX() + VisionConstants.DISTANCE_TOLERANCE;  // forward (meters)
        double isReflected = 1.0;
        if (camToTarget.getY() < 0) isReflected = -1.0;
        double y = isReflected*(Math.abs(camToTarget.getY()) + VisionConstants.DISTANCE_TOLERANCE); // left (meters)
        // double z = camToTarget.getZ();  // up (meters)

        SmartDashboard.putNumber("X Pos", x);
        SmartDashboard.putNumber("Y Pos", y);

        double distance = Math.hypot(x, y);
        return distance;

        // double distance = Math.sqrt(x*x + y*y + z*z);
    }

    public double getHoodAngle(double distance) {
        return hoodMap.get(distance);
    }
}
