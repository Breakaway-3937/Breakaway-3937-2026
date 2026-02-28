// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

public class Vision extends SubsystemBase {

  private final Swerve s_Swerve;
  private final QuestNavSubsystem s_QuestNav;

  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;

  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;

  private final double maxDistance = 5.0;

  private final int[] blueTags = { 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 };
  private final int[] redTags = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

  public Vision(Swerve s_Swerve, QuestNavSubsystem s_QuestNav) {
    this.s_Swerve = s_Swerve;
    this.s_QuestNav = s_QuestNav;

    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2026_field.json"));
    } catch (IOException e) {
      e.printStackTrace();
      layout = new AprilTagFieldLayout(List.of(), 16.518, 8.043);
    }

    leftCamera = new PhotonCamera(Constants.Vision.LEFT_CAMERA);
    rightCamera = new PhotonCamera(Constants.Vision.RIGHT_CAMERA);

    leftEstimator = new PhotonPoseEstimator(layout, Constants.Vision.LEFT_CAMERA_TRANSFORM);
    rightEstimator = new PhotonPoseEstimator(layout, Constants.Vision.RIGHT_CAMERA_TRANSFORM);
  }

  private boolean hasBadTags(EstimatedRobotPose result) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty())
      return false;

    int[] opponentTags = alliance.get().equals(Alliance.Blue) ? redTags : blueTags;

    for (var target : result.targetsUsed) {
      for (int badTag : opponentTags) {
        if (target.getFiducialId() == badTag)
          return true;
      }
    }
    return false;
  }

  private Optional<EstimatedRobotPose> getBestPose(PhotonCamera camera, PhotonPoseEstimator estimator) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    Optional<EstimatedRobotPose> bestPose = Optional.empty();

    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> estimate = estimator.estimateAverageBestTargetsPose(result);

      if (estimate.isEmpty())
        continue;
      if (hasBadTags(estimate.get()))
        continue;

      bestPose = estimate;
    }

    return bestPose;
  }

  public Command setQuestPose() {
    return runOnce(() -> {
      Optional<EstimatedRobotPose> bestResult = Optional.empty();

      Optional<EstimatedRobotPose> leftResult = getBestPose(leftCamera, leftEstimator);
      if (leftResult.isPresent()) {
        bestResult = leftResult;
      } else {
        Optional<EstimatedRobotPose> rightResult = getBestPose(rightCamera, rightEstimator);
        if (rightResult.isPresent()) {
          bestResult = rightResult;
        }
      }

      if (bestResult.isPresent()) {
        Pose2d initialPose = new Pose2d(
            bestResult.get().estimatedPose.getX(),
            bestResult.get().estimatedPose.getY(),
            bestResult.get().estimatedPose.getRotation().toRotation2d());

        s_Swerve.resetPose(initialPose);
        s_QuestNav.setPose(initialPose);

        SmartDashboard.putBoolean("Vision/QuestPoseSuccess", true);
      } else {
        SmartDashboard.putBoolean("Vision/QuestPoseSuccess", false);
      }
    });
  }

  @Override
  public void periodic() {
  }
}