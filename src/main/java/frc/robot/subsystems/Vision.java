// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

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
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Vision extends SubsystemBase {

    private final Field2d turretfield = new Field2d();
    private final Field2d robotfield = new Field2d();

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private static final QuestNav questNav = new QuestNav();

    private static Alliance alliance;

    private static SwerveDriveState swerveState;

    private final Transform3d ROBOT_TO_QUEST = new Transform3d(-0.29, -0.285, 0.26, new Rotation3d(0, 0, -2.356));

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

    // -------------------------------------------------------------------------
    // Double-tap / passthrough detection — translated from 2429's quest.py
    //
    // How it works:
    //   1. We count consecutive periodic loops where QuestNav sends no new frames.
    //   2. Once K_MAX_MISSED_FRAMES is exceeded (≈280 ms at 50 Hz) we assume the
    //      Quest has been double-tapped into passthrough mode and publish the
    //      quest_in_passthrough NetworkTables flag = true.
    //   3. QuestNavADBWatcher (running as a background thread, ideally on the DS)
    //      monitors that flag and fires an ADB command to bring QuestNav back to
    //      the foreground.
    //   4. Once frames start arriving again we clear the flag and log recovery time.
    //
    // NOTE: FRC 2429 intentionally ran the ADB recovery script on their DS laptop
    //       rather than the roboRIO to avoid spawning processes on a real-time OS.
    //       QuestNavADBWatcher.java therefore defaults to being started from the
    //       Robot.java constructor, where you can choose to disable it if you
    //       prefer a purely DS-side solution.  See QuestNavADBWatcher.java.
    // -------------------------------------------------------------------------

    /** Consecutive loops with no new pose frames before we declare a double-tap.
     *  14 loops ≈ 280 ms at 50 Hz — enough to absorb normal NT4 batching delays. */
    private static final int K_MAX_MISSED_FRAMES = 14;

    /** Consecutive loops without a connection before we log a hard disconnect. */
    private static final int K_MAX_DISCONNECTED_COUNT = 50; // ≈1 s at 50 Hz

    private int missedFrameCount     = 0;
    private int disconnectedCount    = 0;
    private int dtapCount            = 0;
    private boolean wasTracking      = false;
    private boolean wasConnected     = false;
    private boolean questHasSynched  = false;
    private double  passthroughStartTime = 0.0;

    // NT entries — watched by QuestNavADBWatcher to trigger ADB recovery
    private final BooleanEntry  questPassthroughEntry;
    private final DoublePublisher dtapCountPublisher;

    /** NT topic key for the passthrough flag (must match QuestNavADBWatcher). */
    public static final String NT_PASSTHROUGH_KEY = "/QuestNav/quest_in_passthrough";
    /** NT topic key for the double-tap counter (informational / logging). */
    public static final String NT_DTAP_COUNT_KEY  = "/QuestNav/quest_dtap_count";

    public Vision(Swerve s_Swerve, Shooter s_Shooter) {
        SmartDashboard.putData("robotField", robotfield);
        SmartDashboard.putData("turretField", turretfield);

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;

        timeOfFlightMap.put(1.384, 0.9);
        timeOfFlightMap.put(2.41, 1.1);
        timeOfFlightMap.put(3.065, 1.32);
        timeOfFlightMap.put(3.6, 1.325);
        timeOfFlightMap.put(4.25, 1.325);
        timeOfFlightMap.put(4.852, 1.4);
        timeOfFlightMap.put(5.412, 1.4);
        timeOfFlightMap.put(6.013, 1.56);
        timeOfFlightMap.put(6.7056, 1.65);
        timeOfFlightMap.put(7.3152, 1.72);
        timeOfFlightMap.put(7.9248, 1.8);

        // --- Passthrough / ADB recovery NT setup ---
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        questPassthroughEntry = inst.getBooleanTopic(NT_PASSTHROUGH_KEY).getEntry(false);
        dtapCountPublisher    = inst.getDoubleTopic(NT_DTAP_COUNT_KEY).publish();

        // Ensure flag starts cleared on robot boot
        questPassthroughEntry.set(false);
        dtapCountPublisher.set(0);
    }

    public void setPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }

    public static boolean isQuestGood() {
        return questNav.isTracking() && questNav.isConnected();
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    public static Pose2d getRobotPose() {
        return filteredPose;
    }

    /** Returns true if the Quest is currently signaling a passthrough condition. */
    public boolean isInPassthrough() {
        return questPassthroughEntry.get();
    }

    /** Returns the total number of double-tap events detected this session. */
    public int getDtapCount() {
        return dtapCount;
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

        expanded = new Rectangle2d(new Pose2d(
                trench.getCenter().getX() + (bufferAway - bufferNear) / 2,
                trench.getCenter().getY(),
                new Rotation2d()),
                trench.getXWidth() + bufferNear + bufferAway,
                trench.getYWidth());

        return expanded.contains(turretPose.getTranslation());
    }

    private static void setTarget() {

        if (alliance == DriverStation.Alliance.Blue) {
            if (isInHomeTerritory()) {
                currentTargetX = 4.6;
                currentTargetY = 4.035;
            } else {
                if (turretPoseY >= 4.035) {
                    currentTargetX = 2.0;
                    currentTargetY = 6.5;
                } else {
                    currentTargetX = 2.0;
                    currentTargetY = 1.5;
                }
            }
        } else {
            if (isInHomeTerritory()) {
                currentTargetX = 11.9;
                currentTargetY = 4.035;
            } else {
                if (turretPoseY >= 4.035) {
                    currentTargetX = 14.5;
                    currentTargetY = 6.5;
                } else {
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
        return () -> {
            if(isInHomeTerritory()) {
                if (s_Shooter.isTracking()) {
                    return s_Shooter.getTurretPosition() < (getAdjustedTurretAngle() - 5 * DEGREE_TO_TURRET)
                            && s_Shooter.getTurretPosition() > (getAdjustedTurretAngle() + 5 * DEGREE_TO_TURRET);
                } else {
                    return true;
                }
            } else {
                if (s_Shooter.isTracking()) {
                    return s_Shooter.getTurretPosition() < (getAdjustedTurretAngle() - 10 * DEGREE_TO_TURRET)
                            && s_Shooter.getTurretPosition() > (getAdjustedTurretAngle() + 10 * DEGREE_TO_TURRET);
                } else {
                    return true;
                }
            }
        };
    }

    public Command setInitPose() {
        return runOnce(() -> {
                if(alliance == DriverStation.Alliance.Blue) {
                    questNav.setPose(new Pose3d(new Pose2d(new Translation2d(3.4044, 4.035), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
                } else {
                    questNav.setPose(new Pose3d(new Pose2d(new Translation2d(12.93288, 4.035), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
                }
        });
    }

    public Command setSpecialInitPose(boolean isLeft) {
        return runOnce(() -> {
            if(alliance == DriverStation.Alliance.Blue) {
                if(isLeft) {
                    questNav.setPose(new Pose3d(new Pose2d(new Translation2d(3.59, 7.225), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
                } else {
                    questNav.setPose(new Pose3d(new Pose2d(new Translation2d(3.59, 0.881), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
                }
            } else {
                if(isLeft) {
                    questNav.setPose(new Pose3d(new Pose2d(new Translation2d(13.0, 0.881), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
                } else {
                    questNav.setPose(new Pose3d(new Pose2d(new Translation2d(13.0, 7.225), new Rotation2d())).transformBy(ROBOT_TO_QUEST));
                }
            }
        });
    }

    @Override
    public void periodic() {

        alliance = DriverStation.getAlliance().orElse(null);

        setTarget();

        swerveState = s_Swerve.getStateCopy();

        questNav.commandPeriodic();
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // --- Process incoming pose frames ---
        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {

                rawRobotPose2d = questFrame.questPose3d().transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();

                double cleanX = xFilter.calculate(rawRobotPose2d.getX());
                double cleanY = yFilter.calculate(rawRobotPose2d.getY());

                filteredRotation = filteredRotation.interpolate(rawRobotPose2d.getRotation(), 0.2);

                filteredPose = new Pose2d(cleanX, cleanY, filteredRotation);

                s_Swerve.addVisionMeasurement(filteredPose, Utils.fpgaToCurrentTime(questFrame.dataTimestamp()));

            } else {
                filteredPose = swerveState.Pose;
            }
        }

        // -----------------------------------------------------------------------
        // Double-tap / passthrough watchdog — translated from 2429's quest.py
        // -----------------------------------------------------------------------
        boolean isConnected = questNav.isConnected();
        boolean isTracking  = questNav.isTracking();

        // Track hard disconnects (NT link dropped entirely)
        if (!isConnected) {
            disconnectedCount++;
            if (disconnectedCount > K_MAX_DISCONNECTED_COUNT && wasConnected) {
                System.out.printf("*** QuestNav connection dropped for %.2fs at %.2fs ***%n",
                        disconnectedCount / 50.0, Timer.getFPGATimestamp());
            }
        } else {
            disconnectedCount = 0;
        }

        // Detect missed frames — these happen when the Quest is in passthrough.
        // questFrames.length == 0 means no new frames arrived this loop.
        // We only care when the Quest is otherwise connected (soft blackout, not hard disconnect).
        if (questFrames.length > 0 && isTracking) {
            // Frames arrived — reset the watchdog
            missedFrameCount = 0;

            // If we were in passthrough and just recovered, clear the flag
            if (questPassthroughEntry.get()) {
                questPassthroughEntry.set(false);
                double recoveryTime = Timer.getFPGATimestamp() - passthroughStartTime;
                System.out.printf("*** QuestNav recovered from passthrough in %.2fs ***%n", recoveryTime);
            }

            questHasSynched = true;

        } else {
            // No new frames this loop
            missedFrameCount++;

            // After K_MAX_MISSED_FRAMES consecutive empty loops, declare passthrough
            if (missedFrameCount > K_MAX_MISSED_FRAMES && wasTracking) {
                passthroughStartTime = Timer.getFPGATimestamp();
                questPassthroughEntry.set(true);
                dtapCount++;
                dtapCountPublisher.set(dtapCount);
                System.out.printf(
                    "*** QuestNav double-tap detected (#%d) at %.2fs — signaling ADB recovery ***%n",
                    dtapCount, passthroughStartTime);
            }
        }

        wasTracking   = isTracking;
        wasConnected  = isConnected;

        /*if(questPassthroughEntry.get() == true) {
            s_Shooter.runServo();
        }*/
        // -----------------------------------------------------------------------

        turretPose = filteredPose.transformBy(ROBOT_TO_TURRET);
        turretPoseX = turretPose.getX();
        turretPoseY = turretPose.getY();
        turretRotation = turretPose.getRotation().getDegrees();

        robotRelativeSpeeds.vxMetersPerSecond = swerveState.Speeds.vxMetersPerSecond;
        robotRelativeSpeeds.vyMetersPerSecond = swerveState.Speeds.vyMetersPerSecond;
        robotRelativeSpeeds.omegaRadiansPerSecond = swerveState.Speeds.omegaRadiansPerSecond;

        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
                swerveState.Pose.getRotation());

        speed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        realDistance = Math.hypot(currentTargetX - turretPoseX, currentTargetY - turretPoseY);
        realAngle = Math.toDegrees(Math.atan2(currentTargetY - turretPoseY, currentTargetX - turretPoseX));

        double targetPhantomX = currentTargetX
                - (fieldRelativeSpeeds.vxMetersPerSecond * (timeOfFlightMap.get(realDistance)));
        double targetPhantomY = currentTargetY
                - (fieldRelativeSpeeds.vyMetersPerSecond * (timeOfFlightMap.get(realDistance)));

        phantomDistance = Math
                .sqrt(Math.pow(targetPhantomX - turretPoseX, 2) + Math.pow(targetPhantomY - turretPoseY, 2));
        phantomAngle = Math.toDegrees(Math.atan2(targetPhantomY - turretPoseY, targetPhantomX - turretPoseX));

        SmartDashboard.putNumber("Phantom Distance", phantomDistance);
        SmartDashboard.putNumber("Distance", realDistance);
        SmartDashboard.putBoolean("Quest In Passthrough", questPassthroughEntry.get());
        SmartDashboard.putNumber("Quest Dtap Count", dtapCount);
        robotfield.setRobotPose(filteredPose);
        turretfield.setRobotPose(turretPose);
    }
}