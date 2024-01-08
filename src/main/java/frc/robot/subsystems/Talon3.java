// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Talon3 extends SubsystemBase {
  private TalonSRX motor7;
  /** Creates a new Talon1. */
  public Talon3() {
    motor7 = new TalonSRX(7);
  } 

public void moveMotor(double speed){
  motor7.set(ControlMode.PercentOutput, speed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
