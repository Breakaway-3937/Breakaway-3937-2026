package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX climberLead, climberFollow;
  private final CANrange eyeOfSauron;
  //private final Follower followerShoulderRequest;
 private final Follower climberFollowerRequest = new Follower(Constants.Climber.CLIMBER_LEAD_CAN_ID, null);
 private final MotionMagicExpoVoltage climberRequest;

  public Climber() {
    climberLead = new TalonFX(Constants.Climber.CLIMBER_LEAD_CAN_ID);
    climberFollow = new TalonFX(Constants.Climber.CLIMBER_FOLLOW_CAN_ID);

    eyeOfSauron = new CANrange(Constants.Climber.EYE_OF_SAURON_CAN_ID);
    //followerShoulderRequest = new Follower(Constants.Climber.INNER_CLIMBER_CAN_ID, false);
    //configMotors();
    configCANranges();
    climberRequest = new MotionMagicExpoVoltage(null);
  }

    public void configMotors() {
    climberFollow.setControl(climberFollowerRequest);
  }

  public void configCANranges() {
    eyeOfSauron.getConfigurator().apply(new CANrangeConfiguration());

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.ProximityThreshold = 0.06;

    eyeOfSauron.getConfigurator().apply(config);
  }

  public Command setClimber() {
    return runOnce(() ->  climberLead.setControl(climberRequest.withPosition(States.ClimberStates.STOW.getClimb())));
  }
 
  @Override
  public void periodic() {

  }

}