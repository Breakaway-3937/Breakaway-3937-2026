package frc.robot.utility;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class Calculations {
    private final Swerve s_Swerve;

    private final Alliance alliance;

    private InterpolatingDoubleTreeMap timeOfFlightMap;

    private final Transform2d robotToTurret = new Transform2d(-0.20, -0.17, new Rotation2d()); // FIXME

    private final Rect trench1 = new Rect(new Point(4.0, 8.1), new Point(5.2, 6.7));
    private final Rect trench2 = new Rect(new Point(4.0, 1.4), new Point(5.2, 0));
    private final Rect trench3 = new Rect(new Point(11.4, 8.1), new Point(12.6, 6.7));
    private final Rect trench4 = new Rect(new Point(11.4, 1.4), new Point(12.6, 0));
    private final Rect[] trenches = { trench1, trench2, trench3, trench4 };

    private final double MAX_POSITIVE_TURRET_ANGLE = 180.0;
    private final double MAX_NEGATIVE_TURRET_ANGLE = -180.0;
    private final double DEGREE_TO_TURRET = -46.92 / 360.0;

    private double robotX;
    private double robotY;
    private double robotAngle;

    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

    private double speed;

    private double realDistance;
    private double realAngle;

    private double phantomDistance;
    private double phantomAngle;

    private double currentTargetX;
    private double currentTargetY;

    public Calculations(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;

        alliance = DriverStation.getAlliance().orElse(null);

        timeOfFlightMap = new InterpolatingDoubleTreeMap();
        timeOfFlightMap.put(5.06, 1.0125);
        timeOfFlightMap.put(4.17, 1.096);
        timeOfFlightMap.put(3.53, 1.06);
        timeOfFlightMap.put(2.79, 1.002);
        timeOfFlightMap.put(1.57, 0.93);
    }

    public boolean isUnderTrench() {
        boolean isUnder = false;
        Pose2d turretPose = s_Swerve.getState().Pose.transformBy(robotToTurret);
        Point turretPoint = new Point(turretPose.getX(), turretPose.getY());

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
            if (robotY >= 4.035 && alliance == DriverStation.Alliance.Red) {
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

    public void runCalculations() {
        robotX = s_Swerve.getState().Pose.getX();
        robotY = s_Swerve.getState().Pose.getY();
        robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();

        SmartDashboard.putNumber("Robot Angle", robotAngle);

        robotRelativeSpeeds.vxMetersPerSecond = s_Swerve.getState().Speeds.vxMetersPerSecond;
        robotRelativeSpeeds.vyMetersPerSecond = s_Swerve.getState().Speeds.vyMetersPerSecond;
        robotRelativeSpeeds.omegaRadiansPerSecond = s_Swerve.getState().Speeds.omegaRadiansPerSecond;

        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, s_Swerve.getState().Pose.getRotation());

        speed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2) + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        realDistance = Math.hypot(currentTargetX - robotX, currentTargetY - robotY);
        realAngle = Math.toDegrees(Math.atan2(currentTargetY - robotY, currentTargetX - robotX));

        double targetPhantomX = currentTargetX - (fieldRelativeSpeeds.vxMetersPerSecond * timeOfFlightMap.get(realDistance));
        double targetPhantomY = currentTargetY - (fieldRelativeSpeeds.vyMetersPerSecond * timeOfFlightMap.get(realDistance));

        phantomDistance = Math.sqrt(Math.pow(targetPhantomX - robotX, 2) + Math.pow(targetPhantomY - robotY, 2));
        phantomAngle = Math.toDegrees(Math.atan2(targetPhantomY - robotY, targetPhantomX - robotX));

        SmartDashboard.putNumber("P Distance", phantomDistance);
        SmartDashboard.putNumber("P Angle", phantomAngle);
        SmartDashboard.putNumber("Real Distance", realDistance);
        SmartDashboard.putNumber("Real Angle", realAngle);
    }

    public double getAdjustedTurretAngle() {
        double adjustedAngle;

        if (speed < 0.1) {
            adjustedAngle = realAngle - robotAngle;
        } else {
            adjustedAngle = phantomAngle - robotAngle;
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

}
