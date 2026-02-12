// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  

  /** Creates a new Vision. */
  public VisionSubsystem() {
    // Constructor - not used yet

    // Establish SmartDashboard Value
    SmartDashboard.putNumber("Yaw", 0);

    // Bind to Network Table
    photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
    targetYaw = photonTable.getEntry("turretcamera/targetYaw");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Read Data from Camera
    // Maybe put on Advantage Scope

    /* 
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()) {

      // Get All the Targets
      List<PhotonTrackedTarget> targets = result.getTargets();

      // Get Best Target
      PhotonTrackedTarget target = getBestTag(targets);

      if (target != null) {
        // Get Yaw
        yaw = target.getYaw();
      }
      

      //System.out.println(target.getFiducialId());

    }
      */

      double yaw = targetYaw.getDouble(0);

      SmartDashboard.putNumber("Yaw", yaw);
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
}
