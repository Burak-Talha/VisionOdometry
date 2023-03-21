// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  PhotonCamera camera;
  Field2d field2d;

  PhotonPoseEstimator photonPoseEstimator;
  Pose3d currentRobotPose;
  Pose2d referencePose;

  Robot(){
    camera = new PhotonCamera("HD_Webcam_C525");
    photonPoseEstimator = new PhotonPoseEstimator(Constants.APRIL_TAG_FIELD_LAYOUT, PoseStrategy.AVERAGE_BEST_TARGETS, camera, new Transform3d());
    field2d = new Field2d();
    referencePose = new Pose2d();
  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    photonPoseEstimator.setReferencePose(referencePose);
  }

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Robot x:", getRobotPose().getX());
    // SmartDashboard.putNumber("Robot y:", getRobotPose().getY());

    //field2d.setRobotPose(getRobotPose().toPose2d()); 
    getRobotPose();
    SmartDashboard.putData(field2d);
  }

  public Pose3d getRobotPose(){
    try{
    PhotonTrackedTarget photonTrackedTarget = camera.getLatestResult().getBestTarget();
    int aprilTagId = photonTrackedTarget.getFiducialId();
    //currentRobotPose = Constants.aprilTags.get(aprilTagId).pose.plus(photonTrackedTarget.getBestCameraToTarget());
    System.out.println("April Tag Id : "+aprilTagId);
    currentRobotPose = Constants.aprilTags.get(aprilTagId).pose.plus(photonTrackedTarget.getBestCameraToTarget());
    System.out.println("X :"+currentRobotPose.getX());
    System.out.println("Y :"+currentRobotPose.getY());
    field2d.setRobotPose(currentRobotPose.getX(), currentRobotPose.getY(), new Rotation2d());
    }catch(Exception exception){
      exception.printStackTrace();
    }
    return currentRobotPose;
  }
}
