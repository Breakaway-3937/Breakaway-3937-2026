package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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

    Transform3d ROBOT_TO_QUEST = new Transform3d(-0.35, 0.01, 0.22, new Rotation3d(0, 0, Math.PI));
    Swerve s_Swerve;
    boolean poseSet = false;

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );

    public QuestNavSubsystem(Swerve swerve) {
        this.s_Swerve = swerve;
        setPose(s_Swerve.getState().Pose);
    }

    public void setPose(Pose2d pose) {
        questNav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
        poseSet = true;
    }

    @Override
    public void periodic() {

        questNav.commandPeriodic();

        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        if(poseSet) {

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
}