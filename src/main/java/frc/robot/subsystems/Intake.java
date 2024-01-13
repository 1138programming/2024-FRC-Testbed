// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
  private CANSparkMax sparkSwivelMotor;
  private CANCoder pivotCaNcoder;
  private CANCoder pivotEncoder;

  /** Creates a new intake. */
  public Intake() {

     sparkSwivelMotor = new CANSparkMax(KIntakeMotorID, MotorType.kBrushless); //CHANGE DEVICE ID LATER
     pivotEncoder = new CANCoder(KIntakeEncoderID);

     sparkSwivelMotor.setIdleMode(IdleMode.kBrake);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinIntake(double speed){
    sparkSwivelMotor.set(speed);
  }
}
