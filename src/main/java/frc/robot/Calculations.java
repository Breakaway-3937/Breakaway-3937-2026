package frc.robot;

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
    private final Rect[] trenches = {trench1, trench2, trench3, trench4};
    private double redTargetX = 11.9;
    private double redTargetY = 4.035;
    private double blueTargetX = 4.6;
    private double blueTargetY = 4.035;

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
        }
        else if (alliance == DriverStation.Alliance.Blue) {
            distance = Math.hypot(blueTargetX - robotX, blueTargetY - robotY);
        }
        else {
            System.out.println("Alliance not recognized");
            distance = 0.0;
        }

        return distance;
    }

    public double getAngleToTarget() {
        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        var alliance = DriverStation.getAlliance().orElse(null);
        double angle;

        if (alliance == DriverStation.Alliance.Red) {
            angle = Math.toDegrees(Math.atan2(redTargetY - robotY, redTargetX - robotX));
        }
        else if (alliance == DriverStation.Alliance.Blue) {
            angle = Math.toDegrees(Math.atan2(blueTargetY - robotY, blueTargetX - robotX));
        }
        else {
            System.out.println("Alliance not recognized");
            return 0.0;
        }

        double robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();
        return angle - robotAngle;
    }

    public boolean isUnderTrench() {
        boolean isUnder = false;
        for(Rect trench : trenches) {
            Point robotPosition = new Point(s_Swerve.getState().Pose.getX(), s_Swerve.getState().Pose.getY());
            if(robotPosition.inside(trench)) {
                isUnder = true;
                break;
            }
        }
        return isUnder;
    }

    public void setTarget(boolean isHub) {
        if(isHub) {
            redTargetX = 5.0;
            redTargetY = 5.0;
            blueTargetX = -5.0;
            blueTargetY = -5.0;
        } 
        else {
            redTargetX = 3.0;
            redTargetY = 3.0;
            blueTargetX = -3.0;
            blueTargetY = -3.0;
        }
    }
}
