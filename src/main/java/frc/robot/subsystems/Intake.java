package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
   private final TalonFX intake, intakePosition;

  public Intake() {
   intake = new TalonFX(Constants.Intake.INTAKE_CAN_ID);
   intakePosition = new TalonFX(Constants.Intake.INTAKE_POSITION_CAN_ID);

  }

  @Override
  public void periodic() {
   
  }

}