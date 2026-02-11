// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  // Create Camera and Bind to Network Table
  PhotonCamera camera = new PhotonCamera("photonvision");

  /** Creates a new Vision. */
  public VisionSubsystem() {
    // Constructor - not used yet

    // Establish SmartDashboard Value
    SmartDashboard.putNumber("Yaw", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Read Data from Camera
    // Maybe put on Advantage Scope

    var result = camera.getLatestResult();

    if (result.hasTargets()) {

      // Get All the Targets
      List<PhotonTrackedTarget> targets = result.getTargets();

      // Get Best Target
      PhotonTrackedTarget target = result.getBestTarget();

      // Get Yaw
      double yaw = target.getYaw();

      SmartDashboard.putNumber("Yaw", yaw);

    }

  }
}
