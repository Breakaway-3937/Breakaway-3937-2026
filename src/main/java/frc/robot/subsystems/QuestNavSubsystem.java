package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();
    private final Transform3d ROBOT_TO_QUEST = new Transform3d(-0.33, 0.01, 0.23, new Rotation3d(0, 0, Math.PI));
    private final Swerve s_Swerve;

    //Here is the Kalman Std Devs.  These are the low trust and high trust values
    private static final Matrix<N3, N1> TRUSTED_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
    private static final Matrix<N3, N1> NOISY_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.9);

    //Here is the Linear Filters for X and Y.  Lets start at a moving average of last 5 frames.  We can adjust from there
    private final LinearFilter xFilter = LinearFilter.movingAverage(5);
    private final LinearFilter yFilter = LinearFilter.movingAverage(5);
    private Rotation2d filteredRotation = new Rotation2d();

    private Pose2d debugPose = new Pose2d(new Translation2d(3.4044, 4.035), new Rotation2d());

    public QuestNavSubsystem(Swerve swerve) {
        this.s_Swerve = swerve;
        setPose(debugPose);
        s_Swerve.resetPose(debugPose); //In front of the blue hub
    }

    public void setPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                // Extract data
                Pose3d rawRobotPose3d = questFrame.questPose3d().transformBy(ROBOT_TO_QUEST.inverse());
                Pose2d rawRobotPose2d = rawRobotPose3d.toPose2d();
                double timestamp = questFrame.dataTimestamp();

                //Apply Smoothing Filters
                double cleanX = xFilter.calculate(rawRobotPose2d.getX());
                double cleanY = yFilter.calculate(rawRobotPose2d.getY());
                // Interpolate handles the 0/360 wrap-around safely
                filteredRotation = filteredRotation.interpolate(rawRobotPose2d.getRotation(), 0.2);
                
                Pose2d filteredPose = new Pose2d(cleanX, cleanY, filteredRotation);

                // Dynamic Kalman Logic (Jump Check)
                // Compare filtered pose against the Swerve's current "best guess"
                Pose2d currentEstimate = s_Swerve.getState().Pose; // Assuming s_Swerve has a getPose()
                double distanceJump = currentEstimate.getTranslation().getDistance(filteredPose.getTranslation());

                Matrix<N3, N1> activeStdDevs;
                if (distanceJump > 1.0) {
                    // If jump is > 1 meter, trust it very little to avoid "teleporting"
                    activeStdDevs = NOISY_STD_DEVS;
                } else {
                    // Trust the Oculus high-precision VIO
                    activeStdDevs = TRUSTED_STD_DEVS;
                }

                s_Swerve.addVisionMeasurement(filteredPose, timestamp, activeStdDevs);
                
                // Logging for tuning
                SmartDashboard.putNumber("QuestNav/Jump Distance", distanceJump);
                SmartDashboard.putNumber("Quest Battery", questNav.getBatteryPercent().getAsInt());
            }
        }
    }
}