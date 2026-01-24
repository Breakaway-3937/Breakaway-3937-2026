package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    QuestNav questNav = new QuestNav();

    Transform3d ROBOT_TO_QUEST = new Transform3d(0, 0, 0.20, new Rotation3d()); // Adjust this transform based on the physical offset between the robot's center and the QuestNav tracking point
    Swerve s_Swerve;

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );

    public QuestNavSubsystem(Swerve swerve, Pose3d initial_pose) {
        questNav.setPose(initial_pose.transformBy(ROBOT_TO_QUEST));
        this.s_Swerve = swerve;
    }

    @Override
    public void periodic() {

        questNav.commandPeriodic();

        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame questFrame : questFrames) {

            if (questFrame.isTracking()) {

                Pose3d questPose = questFrame.questPose3d();

                double timestamp = questFrame.dataTimestamp();

                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                s_Swerve.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }
    }
}