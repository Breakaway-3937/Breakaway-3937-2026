package frc.robot.utility;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Swerve;

public class Calculations {
    private final Swerve s_Swerve;
    private final Rect trench1 = new Rect(new Point(4.0, 8.1), new Point(5.2, 6.7));
    private final Rect trench2 = new Rect(new Point(4.0, 1.4), new Point(5.2, 0));
    private final Rect trench3 = new Rect(new Point(11.4, 8.1), new Point(12.6, 6.7));
    private final Rect trench4 = new Rect(new Point(11.4, 1.4), new Point(12.6, 0));
    private final Rect[] trenches = { trench1, trench2, trench3, trench4 };
    private double redTargetX = 11.9;
    private double redTargetY = 4.035;
    private double blueTargetX = 4.6;
    private double blueTargetY = 4.035;

    private double rotationCount = 0.0;
    private double lastAngle = 0.0;
    private double turretAngleOvershot = 0.0;
    private double turretHome = 30.0;
    private final double DEGREE_TO_TURRET = -46.86 / 360.0;

    public Calculations(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
    }

    public double getDistanceToTarget() {
        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        var alliance = DriverStation.getAlliance().orElse(null);
        double distance;

        if (alliance == DriverStation.Alliance.Red) {
            distance = Math.hypot(redTargetX - robotX, redTargetY - robotY);
        } else if (alliance == DriverStation.Alliance.Blue) {
            distance = Math.hypot(blueTargetX - robotX, blueTargetY - robotY);
        } else {
            System.out.println("Alliance not recognized");
            distance = 0.0;
        }

        //Meters
        return distance;
    }

    public double getAdjustedTurretAngle() {

        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        double robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();

        var alliance = DriverStation.getAlliance().orElse(null);
        double angle;
        double adjustedAngle;

        if (alliance == DriverStation.Alliance.Red) {
            angle = Math.toDegrees(Math.atan2(redTargetY - robotY, redTargetX - robotX));
        } else if (alliance == DriverStation.Alliance.Blue) {
            angle = Math.toDegrees(Math.atan2(blueTargetY - robotY, blueTargetX - robotX));
        } else {
            System.out.println("Alliance not recognized");
            angle = 0.0;
        }

        if (robotAngle > 90 && lastAngle < -90) {
            rotationCount--;
        } else if (robotAngle < -90 && lastAngle > 90) {
            rotationCount++;
        }

        lastAngle = robotAngle;
        adjustedAngle = robotAngle + (rotationCount * 360);

        if (adjustedAngle  > 180 + turretHome + turretAngleOvershot) {
            adjustedAngle += 360;
            rotationCount--;
        } else if (adjustedAngle < -180 + turretHome - turretAngleOvershot) {
            adjustedAngle -= 360;
            rotationCount++;
        }

        return (angle - adjustedAngle) * DEGREE_TO_TURRET;
    }

    public boolean isUnderTrench() {
        boolean isUnder = false;
        for (Rect trench : trenches) {
            Point robotPosition = new Point(s_Swerve.getState().Pose.getX(), s_Swerve.getState().Pose.getY());
            if (robotPosition.inside(trench)) {
                isUnder = true;
                break;
            }
        }
        return isUnder;
    }

    public void setTarget(boolean isHub) {
        if (isHub) {
            redTargetX = 11.9;
            redTargetY = 4.035;
            blueTargetX = 4.6;
            blueTargetY = 4.035;
        } else {
            redTargetX = 0.0;
            redTargetY = 0.0;
            blueTargetX = 0.0;
            blueTargetY = 0.0;
        }
    }

}
