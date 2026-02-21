package frc.robot.utility;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

public class Calculations {
    private final Swerve s_Swerve;
    private final Alliance alliance;
    private final Rect trench1 = new Rect(new Point(4.0, 8.1), new Point(5.2, 6.7));
    private final Rect trench2 = new Rect(new Point(4.0, 1.4), new Point(5.2, 0));
    private final Rect trench3 = new Rect(new Point(11.4, 8.1), new Point(12.6, 6.7));
    private final Rect trench4 = new Rect(new Point(11.4, 1.4), new Point(12.6, 0));
    private final Rect[] trenches = { trench1, trench2, trench3, trench4 };
    private double redTargetX = 11.9;
    private double redTargetY = 4.035;
    private double blueTargetX = 4.6;
    private double blueTargetY = 4.035;
    private final double TURRET_OFFSET_X = -0.13; //Meters
    private final double TURRET_OFFSET_Y = -0.16; //Meters
    private final double MAX_POSITIVE_TURRET_ANGLE = 180.0;
    private final double MAX_NEGATIVE_TURRET_ANGLE = -180.0;

    private final double DEGREE_TO_TURRET = -46.86 / 360.0;

    public Calculations(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        alliance = DriverStation.getAlliance().orElse(null);
    }

    public double getDistanceToTarget() {
        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        
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

        double desiredAngle;
        double angle;

        if (alliance == DriverStation.Alliance.Red) {
            angle = Math.toDegrees(Math.atan2(redTargetY - robotY, redTargetX - robotX));
        } else if (alliance == DriverStation.Alliance.Blue) {
            angle = Math.toDegrees(Math.atan2(blueTargetY - robotY, blueTargetX - robotX));
        } else {
            System.out.println("Alliance not recognized");
            angle = 0.0;
        }

        desiredAngle = angle - robotAngle;

        if(desiredAngle > MAX_POSITIVE_TURRET_ANGLE) {
            desiredAngle -= 360;
        } else if(desiredAngle < MAX_NEGATIVE_TURRET_ANGLE) {
            desiredAngle += 360;
        }

        return desiredAngle * DEGREE_TO_TURRET;
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
        double robotY = s_Swerve.getState().Pose.getY();
        
        if (isHub) {
            redTargetX = 11.9;
            redTargetY = 4.035;
            blueTargetX = 4.6;
            blueTargetY = 4.035;
        } else {
            if(robotY >= 4.035) {
                redTargetX = 15.0;
                redTargetY = 7.25;
                blueTargetX = 1.7;
                blueTargetY = 7.25;
            } else {
                redTargetX = 15.0;
                redTargetY = 0.8;
                blueTargetX = 1.7;
                blueTargetY = 0.8;
            }
        }
    }

}
