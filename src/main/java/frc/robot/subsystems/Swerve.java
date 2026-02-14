package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
	private final Field2d field = new Field2d();
	private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
	private boolean hasAppliedOperatorPerspective = false;
	private final Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
	private final Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

	public Swerve(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
		super(drivetrainConstants, modules);
		SmartDashboard.putData("Field", field);
		configPathplanner();
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	public void configPathplanner() {
		RobotConfig config;

		try {
			config = RobotConfig.fromGUISettings();

			AutoBuilder.configure(
					() -> getState().Pose,
					this::resetPose,
					() -> getState().Speeds,
					(speeds, feedforwards) -> setControl(
							pathApplyRobotSpeeds.withSpeeds(speeds)
									.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
									.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
					new PPHolonomicDriveController(
							new PIDConstants(8, 0, 0),
							new PIDConstants(7, 0, 0)),
					config,
					() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
					this);

			Pathfinding.setPathfinder(new LocalADStar());
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void periodic() {
		field.setRobotPose(getState().Pose);
		if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent((allianceColor) -> {
				setOperatorPerspectiveForward(
						allianceColor == Alliance.Red ? redAlliancePerspectiveRotation
								: blueAlliancePerspectiveRotation);
				hasAppliedOperatorPerspective = true;
			});
		}
	}

	@Override
	public void simulationPeriodic() {
		updateSimState(0.02, 12.0);

	}
}
