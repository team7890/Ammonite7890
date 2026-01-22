// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {

  PhotonCamera objCamera = new PhotonCamera("PhotonLime");
  
  /** Creates a new Photon. */
  public Photon() {
    var result = objCamera.getLatestResult();
    boolean bHasTarget = result.hasTargets();
    List<PhotonTrackedTarget> targetsList = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    // Get information from target.
    double dYaw = target.getYaw();
    double dPitch = target.getPitch();
    double dArea = target.getArea();
    double dSkew = target.getSkew();
    
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    
  
  }

  public void PhotonTrack (){
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
