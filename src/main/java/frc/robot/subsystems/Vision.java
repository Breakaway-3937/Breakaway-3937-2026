// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Vision extends SubsystemBase {

    private final Swerve s_Swerve;
    private final QuestNav questNav = new QuestNav();

    private final Alliance alliance = DriverStation.getAlliance().orElse(null);

    private final Transform3d ROBOT_TO_QUEST = new Transform3d(-0.33, 0.01, 0.23, new Rotation3d(0, 0, Math.PI));
    private final Transform2d ROBOT_TO_TURRET = new Transform2d(-0.15, -0.14, new Rotation2d());

    private final LinearFilter xFilter = LinearFilter.movingAverage(5);
    private final LinearFilter yFilter = LinearFilter.movingAverage(5);

    private Pose2d rawRobotPose2d = new Pose2d();

    private Pose2d filteredPose = new Pose2d();
    private Rotation2d filteredRotation = new Rotation2d();

    private Pose2d turretPose = new Pose2d();
    private Point turretPoint = new Point();

    private final Rect trench1 = new Rect(new Point(4.0, 8.1), new Point(5.2, 6.7));
    private final Rect trench2 = new Rect(new Point(4.0, 1.4), new Point(5.2, 0));
    private final Rect trench3 = new Rect(new Point(11.4, 8.1), new Point(12.6, 6.7));
    private final Rect trench4 = new Rect(new Point(11.4, 1.4), new Point(12.6, 0));
    private final Rect[] trenches = { trench1, trench2, trench3, trench4 };

    private final double MAX_POSITIVE_TURRET_ANGLE = 180.0;
    private final double MAX_NEGATIVE_TURRET_ANGLE = -180.0;
    private final double DEGREE_TO_TURRET = -46.92 / 360.0;

    private InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private double turretPoseX;
    private double turretPoseY;
    private double turretRotation;

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

    private double speed;

    private double realDistance;
    private double realAngle;

    private double phantomDistance;
    private double phantomAngle;

    private double currentTargetX;
    private double currentTargetY;

    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;

    private final PhotonPoseEstimator leftEstimator;
    private final PhotonPoseEstimator rightEstimator;

    private final int[] blueTags = { 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 };
    private final int[] redTags = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

    public Vision(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;

        timeOfFlightMap.put(5.06, 1.0125);
        timeOfFlightMap.put(4.17, 1.096);
        timeOfFlightMap.put(3.53, 1.06);
        timeOfFlightMap.put(2.79, 1.002);
        timeOfFlightMap.put(1.57, 0.93);

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

    public void setPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }

    public Pose2d getFilteredPose() {
        return filteredPose;
    }

    public boolean isUnderTrench() {
        boolean isUnder = false;

        for (Rect trench : trenches) {
            if (turretPoint.inside(trench)) {
                isUnder = true;
                break;
            }
        }
        return isUnder;
    }

    public void setTarget(boolean isHub) {

        if (isHub && alliance == DriverStation.Alliance.Red) {
            // Red Hub
            currentTargetX = 11.9;
            currentTargetY = 4.035;
        } else if (isHub && alliance == DriverStation.Alliance.Blue) {
            // Blue Hub
            currentTargetX = 4.6;
            currentTargetY = 4.035;
        } else {
            if (turretPoseY >= 4.035 && alliance == DriverStation.Alliance.Red) {
                // Red Lob
                currentTargetX = 15.0;
                currentTargetY = 7.25;
            } else {
                // Blue Lob
                currentTargetX = 1.7;
                currentTargetY = 0.8;
            }
        }
    }

    public double getAdjustedTurretAngle() {
        double adjustedAngle;

        if (speed < 0.1) {
            adjustedAngle = realAngle - turretRotation;
        } else {
            adjustedAngle = phantomAngle - turretRotation;
        }

        if (adjustedAngle > MAX_POSITIVE_TURRET_ANGLE) {
            adjustedAngle -= 360;
        } else if (adjustedAngle < MAX_NEGATIVE_TURRET_ANGLE) {
            adjustedAngle += 360;
        }

        SmartDashboard.putNumber("Adjusted Angle", adjustedAngle);

        return adjustedAngle * DEGREE_TO_TURRET;
    }

    public double getAdjustedDistance() {
        double adjustedDistance;

        if (speed < 0.1) {
            adjustedDistance = realDistance;
        } else {
            adjustedDistance = phantomDistance;
        }

        SmartDashboard.putNumber("Adjusted Distance", adjustedDistance);

        return adjustedDistance;
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

    public Command setInitPose() {
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
                setPose(initialPose);

                SmartDashboard.putBoolean("Vision/InitPoseSuccess", true);
            } else {
                SmartDashboard.putBoolean("Vision/InitPoseSuccess", false);
            }
        });
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {

                rawRobotPose2d = questFrame.questPose3d().transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();

                double cleanX = xFilter.calculate(rawRobotPose2d.getX());
                double cleanY = yFilter.calculate(rawRobotPose2d.getY());

                filteredRotation = filteredRotation.interpolate(rawRobotPose2d.getRotation(), 0.2);

                filteredPose = new Pose2d(cleanX, cleanY, filteredRotation);

                double distanceJump = rawRobotPose2d.getTranslation().getDistance(filteredPose.getTranslation());

                SmartDashboard.putNumber("QuestNav/Jump Distance", distanceJump);
                SmartDashboard.putNumber("Quest Battery", questNav.getBatteryPercent().getAsInt());
            }
        }

        turretPose = filteredPose.transformBy(ROBOT_TO_TURRET);
        turretPoseX = turretPose.getX();
        turretPoseY = turretPose.getY();
        turretRotation = turretPose.getRotation().getDegrees();

        robotRelativeSpeeds.vxMetersPerSecond = s_Swerve.getState().Speeds.vxMetersPerSecond;
        robotRelativeSpeeds.vyMetersPerSecond = s_Swerve.getState().Speeds.vyMetersPerSecond;
        robotRelativeSpeeds.omegaRadiansPerSecond = s_Swerve.getState().Speeds.omegaRadiansPerSecond;

        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
                s_Swerve.getState().Pose.getRotation());

        speed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        realDistance = Math.hypot(currentTargetX - turretPoseX, currentTargetY - turretPoseY);
        realAngle = Math.toDegrees(Math.atan2(currentTargetY - turretPoseY, currentTargetX - turretPoseX));

        double targetPhantomX = currentTargetX
                - (fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlightMap.get(realDistance));
        double targetPhantomY = currentTargetY
                - (fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlightMap.get(realDistance));

        phantomDistance = Math
                .sqrt(Math.pow(targetPhantomX - targetPhantomX, 2) + Math.pow(targetPhantomY - targetPhantomY, 2));
        phantomAngle = Math.toDegrees(Math.atan2(targetPhantomY - targetPhantomY, targetPhantomX - targetPhantomX));
    }
}
