// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sparkmax1 extends SubsystemBase {
  private CANSparkMax motor1;

  /** Creates a new Sparkmax1. */
  public Sparkmax1() { 
    motor1 = new CANSparkMax(1, MotorType.kBrushless);
  }
public void movemotor(double speed) {
  motor1.set(speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}