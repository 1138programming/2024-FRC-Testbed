// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.getEncoder;

// import for CANSparkMax
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// import for Encoder
import com.revrobotics.RelativeEncoder;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;

// import for talonfx6
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase {
    
  private CANSparkMax intakeDeployMotor;
  private RelativeEncoder intakeDeployEncoder;
  private TalonFX intakeSpinMotor;
  // check to see it is TalonFX 5 or TalonFX 6;
  private DigitalInput intakeTopLimit;
  private DigitalInput intakeBottomLimit;

  private double setSpeed;
  private double setVelocity;

  /** Creates a new intake. */
  public Intake() {

    intakeDeployMotor = new CANSparkMax(KIntakeDeployMotorID, MotorType.kBrushless); //CHANGE DEVICE ID LATER
    intakeDeployEncoder = intakeDeployMotor.getEncoder(null, KIntakeDeployEncoderID);
    //  intakeDeployEncoder = new RelativeEncoder(KIntakeDeployEncoderID);
     intakeDeployMotor.setIdleMode(IdleMode.kBrake);
     // TalonFX motor = new TalonFX(0); 
     // intake roller, intake deploy

     intakeSpinMotor = new TalonFX(KIntakeSpinMotorID);

     intakeTopLimit = new DigitalInput(KIntakeTopLimitID);
     intakeBottomLimit = new DigitalInput(KIntakeBottomLimitID);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // intakeDeployEncoder
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
  

  // intakeDeployMotor
  public void moveIntakeUp(double speed){
    intakeDeployMotor.set(speed);
  }

  public void moveIntakeDown(double speed){
    intakeDeployMotor.set(-speed);
  }

  public void intakeStop(double speed){
    intakeDeployMotor.set(0);
  }

  public void moveIntakeUpToSpeed(){
    intakeDeployMotor.set(setSpeed);
  }

  public double getTargetVelocity(){
    return setVelocity;
  }

  public void moveIntakeStop(){
    intakeDeployMotor.set(0);
  }

  public void setIntakeSpeed(double speed, double velocity){
    this.setSpeed = speed;
    this.setVelocity = velocity;
  }

 
  // intakeSpinMotor
  // three methods: forwards, backwards, stop
  public void SpinIntakeForwards(){
    // motor.set(TalonFX.PercentOutput, 0.3); 
    intakeSpinMotor.set(100);
  }

  public void spinIntakeBackwards(){
    // motor.set(TalonFX.PercentOutput, -0.3); 
    intakeSpinMotor.set(-100);
  }

  public void SpinIntakeStop(){
    // motor.set(TalonFX.PercentOutput, 0);
    intakeSpinMotor.set(0);
  }
  
}
