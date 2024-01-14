// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import for CANSparkMax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import for Encoder
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;

// import for TalonFX

public class Intake extends SubsystemBase {
    
  private CANSparkMax intakeDeployMotor;
  private RelativeEncoder intakeDeployEncoder;
  // private TalonFX intakeSpinMotor;
  // check to see it is TalonFX 5 or TalonFX 6;
  private DigitalInput intakeTopLimit;
  private DigitalInput intakeBottomLimit;

  private double setSpeed;
  private double setVelocity;

  /** Creates a new intake. */
  public Intake() {

     intakeDeployMotor = new CANSparkMax(KIntakeDeployMotorID, MotorType.kBrushless); //CHANGE DEVICE ID LATER
     //intakeDeployEncoder = new CANCoder(KIntakeDeployEncoderID);
     intakeDeployEncoder = new RelativeEncoder(KIntakeDeployEncoderID);
     intakeDeployMotor.setIdleMode(IdleMode.kBrake);
     // TalonFX motor = new TalonFX(0); 
     // intake roller, intake deploy

     intakeTopLimit = new DigitalInput(KIntakeTopLimitID);
     intakeBottomLimit = new DigitalInput(KIntakeBottomLimitID);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Encoder
  public double getIntakePosition(){
    return intakeDeployEncoder.getPosition();
  }

  public void setIntakePosUp(double intakePosUp){
    intakeDeployEncoder.setPosition(intakePosUp);
  }

  public void setIntakePosDown(double intakePosDown){
    intakeDeployEncoder.setPosition(intakePosDown);
  }

  public boolean getTopLimitSwitch(){
    return !intakeTopLimit.get();
  }

  public boolean getBottomLimitSwitch(){
    return intakeBottomLimit.get();
  }

  public void intakeStop(){
    intakeDeployMotor.set(ControlMode.PercentOutput, 0);
  }

  

  // CANSparkMax
  public void moveIntakeUp(double speed){
    intakeDeployMotor.set(speed);
  }

  public void moveIntakeDown(double speed){
    intakeDeployMotor.set(speed);
  }

  public void moveIntakeUpToSpeed(){
    intakeDeployMotor.set(setSpeed);
  }

  public double getTargetVelocity(){
    return setVelocity;
  }

  public void setIntakeSpeed(double speed, double velocity){
    this.setSpeed = speed;
    this.setVelocity = velocity;
  }

 
  // TalonFX
  // three methods: forwards, backwards, stop
  public void SpinIntakeForwards(double speed){
    // motor.set(TalonFX.PercentOutput, 0.3); 
  }

  public void spinIntakeBackwards(double speed){
    // motor.set(TalonFX.PercentOutput, -0.3); 
  }

  public void SpinIntakeStop(double speed){
    // motor.set(TalonFX.PercentOutput, 0);
  }
  
}
