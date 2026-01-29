package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private final Swerve s_Swerve;
    private final double redTargetX = 5.0;
    private final double redTargetY = 5.0;
    private final double blueTargetX = -5.0;
    private final double blueTargetY = -5.0;

    public Vision(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
    }

    public double getDistance() {
        double robotX = s_Swerve.getState().Pose.getX();
        double robotY = s_Swerve.getState().Pose.getY();
        var alliance = DriverStation.getAlliance().orElse(null);

        if (alliance == DriverStation.Alliance.Red) {
            return Math.hypot(redTargetX - robotX, redTargetY - robotY);
        }
        else if (alliance == DriverStation.Alliance.Blue) {
            return Math.hypot(blueTargetX - robotX, blueTargetY - robotY);
        }
        else {
            System.out.println("Alliance not recognized");
            return 0.0;
        }
    }
}
