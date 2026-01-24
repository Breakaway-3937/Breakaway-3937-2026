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

public class Climber extends SubsystemBase {
   private final TalonFX claw1, claw2;

  public Climber() {
    claw1 = new TalonFX(Constants.Climber.CLAW1_CAN_ID);
    claw2 = new TalonFX(Constants.Climber.CLAW2_CAN_ID);
  }

  @Override
  public void periodic() {
   
  }

}