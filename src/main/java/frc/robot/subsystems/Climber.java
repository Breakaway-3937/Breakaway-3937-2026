package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX outerClimber, innerClimber;
  private final CANrange eyeOfSauron;

  public Climber() {
    outerClimber = new TalonFX(Constants.Climber.OUTER_CLIMBER_CAN_ID);
    innerClimber = new TalonFX(Constants.Climber.INNER_CLIMBER_CAN_ID);
    eyeOfSauron = new CANrange(Constants.Climber.EYE_OF_SAURON_CAN_ID);

    configCANranges();
  }

  public void configCANranges() {
    eyeOfSauron.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.06;

    eyeOfSauron.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {

  }

}