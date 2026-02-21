package frc.robot.utility;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

    private double currentTargetX;
    private double currentTargetY;

    private final double MAX_POSITIVE_TURRET_ANGLE = 180.0;
    private final double MAX_NEGATIVE_TURRET_ANGLE = -180.0;

    private final double DEGREE_TO_TURRET = -46.86 / 360.0;

    private InterpolatingDoubleTreeMap timeOfFlightMap;

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

    public double getDistanceToTarget() {
        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();

        double distance = Math.hypot(currentTargetX - robotX, currentTargetY - robotY);

        //Meters
        return distance;
    }

    /*public double getAdjustedTurretAngle() {

        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        

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
    }*/

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
        
        if (isHub && alliance == DriverStation.Alliance.Red) {
            //Red Hub
            currentTargetX = 11.9;
            currentTargetY = 4.035;
        } else if (isHub && alliance == DriverStation.Alliance.Blue) {
            //Blue Hub
            currentTargetX = 4.6;
            currentTargetY = 4.035;
        } else {
            if(robotY >= 4.035 && alliance == DriverStation.Alliance.Red) {
                //Red Lob
                currentTargetX = 15.0;
                currentTargetY = 7.25;
            } else {
                //Blue Lob
                currentTargetX = 1.7;
                currentTargetY = 0.8;
            }
        }
    }

    public double shootOnTheMove() {
        double robotVelocityX = s_Swerve.getState().Speeds.vxMetersPerSecond;
        double robotVelocityY = s_Swerve.getState().Speeds.vyMetersPerSecond;
        double robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();

        double targetPhantomX = currentTargetX - (robotVelocityX * timeOfFlightMap.get(getDistanceToTarget()));
        double targetPhantomY = currentTargetY - (robotVelocityY * timeOfFlightMap.get(getDistanceToTarget()));

        double angle = Math.sqrt(Math.pow(targetPhantomX, 2) + Math.pow(targetPhantomY, 2));
        angle = Math.toDegrees(Math.atan2(targetPhantomY, targetPhantomX));

        double desiredAngle = angle - robotAngle;

        if(desiredAngle > MAX_POSITIVE_TURRET_ANGLE) {
            desiredAngle -= 360;
        } else if(desiredAngle < MAX_NEGATIVE_TURRET_ANGLE) {
            desiredAngle += 360;
        }

        return desiredAngle * DEGREE_TO_TURRET;
    }

}
