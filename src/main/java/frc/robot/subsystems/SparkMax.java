// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.KSpark3ID;
import static frc.robot.Constants.KSpark4ID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMax extends SubsystemBase {
  private CANSparkMax motor1;
  private CANSparkMax motor2;

  private RelativeEncoder motor1Encoder;
  private RelativeEncoder motor2Encoder;

  /** Creates a new SparkMax. */
  public SparkMax() {
    motor1 = new CANSparkMax(KSpark3ID, MotorType.kBrushless);
    motor2 = new CANSparkMax(KSpark4ID, MotorType.kBrushless);

    motor1Encoder = motor1.getEncoder();
    motor2Encoder = motor2.getEncoder();
  }

  public void moveMotor1(double speed) {
    motor1.set(speed);
  }

  public void moveMotor2(double speed) {
    motor2.set(speed);
  }

  public void moveBothMotors(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("sparkmax 1 speed", motor1Encoder.getVelocity());
    SmartDashboard.putNumber("sparkmax 2 speed", motor2Encoder.getVelocity());
  }
}
