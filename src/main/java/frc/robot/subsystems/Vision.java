// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Vision extends SubsystemBase {

    private final Field2d turretfield = new Field2d();
    private final Field2d robotfield = new Field2d();

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final QuestNav questNav = new QuestNav();

    private static Alliance alliance;

    private final Transform3d ROBOT_TO_QUEST;

    private final Transform2d ROBOT_TO_TURRET = new Transform2d(-0.15, 0.14, new Rotation2d());

    private final LinearFilter xFilter = LinearFilter.movingAverage(5);
    private final LinearFilter yFilter = LinearFilter.movingAverage(5);

    private Pose2d rawRobotPose2d = new Pose2d();

    private static Pose2d filteredPose = new Pose2d();
    private Rotation2d filteredRotation = new Rotation2d();

    private static Rectangle2d expanded = new Rectangle2d(new Translation2d(), new Translation2d());

    private final static Rectangle2d trench1 = new Rectangle2d(new Translation2d(3.7, 8.1),
            new Translation2d(5.5, 6.7));
    private final static Rectangle2d trench2 = new Rectangle2d(new Translation2d(3.7, 1.4), new Translation2d(5.5, 0));
    private final static Rectangle2d trench3 = new Rectangle2d(new Translation2d(11.1, 8.1),
            new Translation2d(12.9, 6.7));
    private final static Rectangle2d trench4 = new Rectangle2d(new Translation2d(11.1, 1.4),
            new Translation2d(12.9, 0));
    // Added 0.3 to the trench zone
    private final static Rectangle2d[] trenches = { trench1, trench2, trench3, trench4 };

    private final static double MAX_POSITIVE_TURRET_ANGLE = 180.0;
    private final static double MAX_NEGATIVE_TURRET_ANGLE = -180.0;
    private final static double DEGREE_TO_TURRET = -46.92 / 360.0;

    private InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static Pose2d turretPose = new Pose2d();
    private static double turretPoseX;
    private static double turretPoseY;
    private static double turretRotation;

    private static final double trenchLookAheadSeconds = 0.5;
    private static final double trenchMinBuffer = 0.0;

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private static ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

    private static double speed;

    private static double realDistance;
    private static double realAngle;

    private static double phantomDistance;
    private static double phantomAngle;

    private static double currentTargetX;
    private static double currentTargetY;

    private Pose2d initialPose;

    public Vision(Swerve s_Swerve, Shooter s_Shooter) {
        SmartDashboard.putData("robotField", robotfield);
        SmartDashboard.putData("turretField", turretfield);

        ROBOT_TO_QUEST = (Constants.COMPBOT)
                ? new Transform3d(-0.33, 0.0, 0.28, new Rotation3d(0, 0, Math.PI))
                : new Transform3d(-0.33, 0.01, 0.23, new Rotation3d(0, 0, Math.PI));

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;

        timeOfFlightMap.put(5.06, 1.0125);
        timeOfFlightMap.put(4.17, 1.096);
        timeOfFlightMap.put(3.53, 1.06);
        timeOfFlightMap.put(2.79, 1.002);
        timeOfFlightMap.put(1.57, 0.93);

        questNav.setPose(new Pose3d(new Pose2d(new Translation2d(3.4044, 4.035), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
        
        try {
            Thread.sleep(1000);
        } catch (Exception e) {
            System.out.print(e);
        };

        questNav.setPose(new Pose3d(new Pose2d(new Translation2d(3.4044, 4.035), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
    }

    public void setPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    public static Pose2d getRobotPose() {
        return filteredPose;
    }

    public static boolean isInHomeTerritory() {
        boolean isHome = false;

        if (alliance == DriverStation.Alliance.Blue) {
            if (turretPoseX <= 4.6) {
                isHome = true;
            }
        } else {
            if (turretPoseX >= 11.9) {
                isHome = true;
            }
        }

        return isHome;
    }

    public static boolean isUnderTrench() {
        boolean isUnder = false;

        for (Rectangle2d trench : trenches) {
            if (isDynamicTrenchHit(trench)) {
                isUnder = true;
                break;
            }
        }

        return isUnder;
    }

    private static boolean isDynamicTrenchHit(Rectangle2d trench) {

        double vx = fieldRelativeSpeeds.vxMetersPerSecond;

        double bufferAway = trenchMinBuffer + Math.max(0, -vx) * trenchLookAheadSeconds;
        double bufferNear = trenchMinBuffer + Math.max(0, vx) * trenchLookAheadSeconds;
        //double buffer = 2 * (Math.abs(vx) * trenchLookAheadSeconds);

        expanded = new Rectangle2d(new Pose2d(
                trench.getCenter().getX() + (bufferAway - bufferNear) / 2, 
                trench.getCenter().getY(), 
                new Rotation2d()
            ), 
            trench.getXWidth() + bufferNear + bufferAway, 
            trench.getYWidth());

        return expanded.contains(turretPose.getTranslation());
    }

    private static void setTarget() {

        if (alliance == DriverStation.Alliance.Blue) {
            if (isInHomeTerritory()) {
                // Blue Hub
                currentTargetX = 4.6;
                currentTargetY = 4.035;
            } else {
                if (turretPoseY >= 4.035) {
                    // Blue High Lob
                    currentTargetX = 2.0;
                    currentTargetY = 6.5;
                } else {
                    // Blue Low Lob
                    currentTargetX = 2.0;
                    currentTargetY = 1.5;
                }
            }
        } else {
            if (isInHomeTerritory()) {
                // Red Hub
                currentTargetX = 11.9;
                currentTargetY = 4.035;
            } else {
                if (turretPoseY >= 4.035) {
                    // Red High Lob
                    currentTargetX = 14.5;
                    currentTargetY = 6.5;
                } else {
                    // Red Low Lob
                    currentTargetX = 14.5;
                    currentTargetY = 1.5;
                }
            }
        }
    }

    public static double getAdjustedTurretAngle() {
        double adjustedAngle;

        if (speed < 0.001) {
            adjustedAngle = realAngle - turretRotation;
        } else {
            adjustedAngle = phantomAngle - turretRotation;
        }

        if (adjustedAngle > MAX_POSITIVE_TURRET_ANGLE) {
            adjustedAngle -= 360;
        } else if (adjustedAngle < MAX_NEGATIVE_TURRET_ANGLE) {
            adjustedAngle += 360;
        }

        return adjustedAngle * DEGREE_TO_TURRET;
    }

    public static double getAdjustedDistance() {
        double adjustedDistance;

        if (speed < 0.001) {
            adjustedDistance = realDistance;
        } else {
            adjustedDistance = phantomDistance;
        }

        return adjustedDistance;
    }

    public BooleanSupplier isTurretSafe() {
        return () -> s_Shooter.getTurretPosition() < (getAdjustedTurretAngle() - 5 * DEGREE_TO_TURRET)
                && s_Shooter.getTurretPosition() > (getAdjustedTurretAngle() + 5 * DEGREE_TO_TURRET);
    }

    public boolean isInitPoseSet() {
        return filteredPose.equals(initialPose);
    }

    @Override
    public void periodic() {

        alliance = DriverStation.getAlliance().orElse(null);

        setTarget();
        
        questNav.commandPeriodic();
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {

                rawRobotPose2d = questFrame.questPose3d().transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();

                double cleanX = xFilter.calculate(rawRobotPose2d.getX());
                double cleanY = yFilter.calculate(rawRobotPose2d.getY());

                filteredRotation = filteredRotation.interpolate(rawRobotPose2d.getRotation(), 0.2);

                filteredPose = new Pose2d(cleanX, cleanY, filteredRotation);

            } else {
                filteredPose = s_Swerve.getState().Pose;
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
                - (fieldRelativeSpeeds.vxMetersPerSecond * (timeOfFlightMap.get(realDistance) + 0.1));
        double targetPhantomY = currentTargetY
                - (fieldRelativeSpeeds.vyMetersPerSecond * (timeOfFlightMap.get(realDistance) + 0.1));

        phantomDistance = Math
                .sqrt(Math.pow(targetPhantomX - turretPoseX, 2) + Math.pow(targetPhantomY - turretPoseY, 2));
        phantomAngle = Math.toDegrees(Math.atan2(targetPhantomY - turretPoseY, targetPhantomX - turretPoseX));

        SmartDashboard.putBoolean("Is Under Trench", isUnderTrench());
        SmartDashboard.putNumber("Phantom Distance", phantomDistance);
        SmartDashboard.putNumber("VX", fieldRelativeSpeeds.vxMetersPerSecond);
        robotfield.setRobotPose(filteredPose);
        turretfield.setRobotPose(turretPose);
    }
}
