package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final Swerve s_Swerve;
    private double redTargetX = 5.0;
    private double redTargetY = 5.0;
    private double blueTargetX = -5.0;
    private double blueTargetY = -5.0;

    public Vision(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
    }

    public double getDistance() {
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

    public double getAngle() {
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
