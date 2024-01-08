// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Talon2 extends SubsystemBase {
  private TalonSRX motor6;
  /** Creates a new Talon1. */
  public Talon2() {
    motor6 = new TalonSRX(6);
  } 

public void moveMotor(double speed){
  motor6.set(ControlMode.PercentOutput, speed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
