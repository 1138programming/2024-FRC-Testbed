// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.revrobotics.CANSparkFlex.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  
  private CANSparkFlex flywheelMotor;
  
  private CANSparkFlex flywheelIndexerMotor;

private double setSpeed;
private double setVelocity;

  private CANCoder flywheelCanCoder;

  public Flywheel() {

    flywheelMotor = new CANSparkFlex(KFlyWheelMotorID, MotorType.kBrushless);

    flywheelCanCoder = new CANCoder(KFlywheelEncoderID);

    flywheelMotor.setIdleMode(IdleMode.kBrake);
  }
  public double getSpinEncoder() {
    return flywheelCanCoder.getAbsolutePosition();
}
public double getSpinEncoderVelocity(){
  return flywheelCanCoder.getVelocity();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void SpinFlywheel(double speed){
  flywheelMotor.set(speed);
}
public void spinFlywheelToSpeed(){
  flywheelMotor.set(setSpeed);
}
public double getTargetVelocity(){
  return setVelocity;
}
public void setFlywheelSpeed(double speed, double velocity){
  this.setSpeed = speed;
  this.setVelocity = velocity;
}

 public void SpinFlywheelIndexer(double speed){
  flywheelIndexerMotor.set(speed);
 }

public double getEncoderRaw() {
  return flywheelCanCoder.getAbsolutePosition() * KFlywheelEncoderRatio;
}
public void spinFlywheelToPos(double setPoint){

}
public void encoder(double KFlywheelEncoder){

  double RPMOutput = 0.0;

  double rawOutput = RPMOutput * (4096/360);

  // 4096 encoder counts per rev, on robotics, description
  //dont have speed info, example CHANGE LATER!!!!

  SmartDashboard.putBoolean("Flywheel Spinning", true);
  flywheelMotor.set(rawOutput);

}
}
