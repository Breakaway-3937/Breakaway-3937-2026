package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

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

    Transform3d ROBOT_TO_QUEST = new Transform3d(0, 0, 0.20, new Rotation3d(0,0,0));
    Swerve s_Swerve;

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );

    public QuestNavSubsystem(Swerve swerve) {
        this.s_Swerve = swerve;
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

                Pose3d questPose = questFrame.questPose3d();

                double timestamp = questFrame.dataTimestamp();

                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
                
                double ctreTime = Utils.fpgaToCurrentTime(timestamp);
                
                s_Swerve.addVisionMeasurement(robotPose.toPose2d(), ctreTime, QUESTNAV_STD_DEVS);
            }
        }
    }
}