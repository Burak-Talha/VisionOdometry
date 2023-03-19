// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
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
  EstimatedRobotPose currentRobotPose;
  Pose2d referencePose;

  Robot(){
    camera = new PhotonCamera("photonvision");
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
    currentRobotPose = photonPoseEstimator.update().get();
    field2d.setRobotPose(currentRobotPose.estimatedPose.toPose2d());

    SmartDashboard.putData("Robot Pose :", field2d);
    SmartDashboard.putNumber("Robot X : ", currentRobotPose.estimatedPose.getX());
    SmartDashboard.putNumber("Robot Y : ", currentRobotPose.estimatedPose.getY());
    SmartDashboard.putNumber("Robot Heading : ", currentRobotPose.estimatedPose.getRotation().getAngle());
  }
}
