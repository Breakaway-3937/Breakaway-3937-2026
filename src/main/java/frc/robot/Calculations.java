package frc.robot;

import java.io.ObjectInputFilter.Config;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double adjustedAngle = 0.0;
    private double turretAngleOvershot = 0.0;
    private double turretHome = 30.0;
    
        boolean flopplyFlag = false;

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

        return distance;
    }

    public void setTurretAngle(TalonFX turret, MotionMagicExpoVoltage turretRequest) {
        MotionMagicConfigs normalConfig = new MotionMagicConfigs();
        normalConfig.MotionMagicExpo_kV = 0.3;
        normalConfig.MotionMagicExpo_kA = 0.1;
        MotionMagicConfigs fastConfig = new MotionMagicConfigs();
        fastConfig.MotionMagicExpo_kV = 0.02;
        fastConfig.MotionMagicExpo_kA = 0.02;

        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        var alliance = DriverStation.getAlliance().orElse(null);
        double angle;

        if (alliance == DriverStation.Alliance.Red) {
            angle = Math.toDegrees(Math.atan2(redTargetY - robotY, redTargetX - robotX));
            SmartDashboard.putNumber("MY RED ANGLE", angle);
        } else if (alliance == DriverStation.Alliance.Blue) {
            angle = Math.toDegrees(Math.atan2(blueTargetY - robotY, blueTargetX - robotX));
            SmartDashboard.putNumber("My BLUE ANGLE", angle);
        } else {
            System.out.println("Alliance not recognized");
            angle = 0.0;
        }

        double robotAngle = s_Swerve.getState().Pose.getRotation().getDegrees();

        if (robotAngle > 90 && lastAngle < -90) {
            rotationCount--;
        } else if (robotAngle < -90 && lastAngle > 90) {
            rotationCount++;
        }
        lastAngle = robotAngle;
        adjustedAngle = robotAngle + (rotationCount * 360);
        SmartDashboard.putNumber("Adjusted Angle", adjustedAngle);
        SmartDashboard.putNumber("Rotation Count", rotationCount);
        SmartDashboard.putNumber("Last Angle", lastAngle);
        SmartDashboard.putNumber("Robot Angle", robotAngle);
        SmartDashboard.putNumber("Angle to Target", angle);
        if (adjustedAngle - (360 * rotationCount) + turretAngleOvershot > 180 + turretHome + turretAngleOvershot) {
            adjustedAngle += 360;
            flopplyFlag=false;
            System.out.println("Flip Positive");
            turret.getConfigurator().apply(fastConfig);
            turret.setControl(turretRequest.withPosition((angle - adjustedAngle) * DEGREE_TO_TURRET).withSlot(1));
        } else if (adjustedAngle - (360 * rotationCount) + turretAngleOvershot < -180 + turretHome - turretAngleOvershot) {
            adjustedAngle -= 360;
            flopplyFlag = false;
            System.out.println("Flip Negative");
            turret.getConfigurator().apply(fastConfig);
            turret.setControl(turretRequest.withPosition((angle - adjustedAngle) * DEGREE_TO_TURRET).withSlot(1));
        } else {
            if (!flopplyFlag) {
                System.out.println("No Flip");
                flopplyFlag = true;
            }
            
            turret.getConfigurator().apply(normalConfig);
            turret.setControl(turretRequest.withPosition((angle - adjustedAngle) * DEGREE_TO_TURRET).withSlot(0));
        }
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

    public double getAngle() {
        return s_Swerve.getState().Pose.getRotation().getDegrees();
    }
}
