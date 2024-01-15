// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
//import static frc.robot.Constants.kHangEncoderValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;

import static frc.robot.Constants.KHangBottomLSID;
import static frc.robot.Constants.KHangEncoderPortID;
import static frc.robot.Constants.KHangTopLSID;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.CAN;
// add a limit switch
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hang extends SubsystemBase {
  /** Creates a new Hang. */
  //Hang Motor
  private CANSparkMax hangMotor;
  //Hang Encoder
  private RelativeEncoder hangEncoder;
  // Hang Limit Switchs
  private DigitalInput hangTopLS;
  private DigitalInput hangBottomLS;
  
  public Hang(int hangMotorID, boolean hangMotorReversed) {
    //Motor
    hangMotor = new CANSparkMax(hangMotorID, MotorType.kBrushless);
    hangEncoder = new RelativeEncoder(null, KHangEncoderPortID);
   // CANSparkMax.getEncoder(null, KHangEncoderPortID);
    //CANSparkMax.getEncoder(KHangEncoderPortID, 380);

    hangMotor.setIdleMode(IdleMode.kBrake);
    //this.hangMotor.setInverted(hangMotorReversed);
    // only use if necessary 
    hangTopLS = new DigitalInput(KHangTopLSID);
    hangBottomLS = new DigitalInput(KHangBottomLSID);
    }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  
  public double getHangPosition(){
    return hangEncoder.getPosition();
  }
  
  //public void forwardToEncoder(double hangEncoder){
    //hangMotor.set(ControlMode.PercentOutput, 1);
    //}
  

  public void setHangPosUp(double HangPosUp){
    hangEncoder.setPosition(HangPosUp);

  }
  //Create constants for set positions
  public void setHangPosDown(double HangPosDown){
    hangEncoder.setPosition(HangPosDown);
  }

  public void openHang (double speed){
    hangMotor.set(speed);

  }

  public void closeHang(double speed){
    hangMotor.set(-speed);
  }

  public boolean getHangTopLS(){
    return hangTopLS.get();
  }
  public boolean getHangBottomLS(){
    return hangBottomLS.get();
  }
  //Limit Switches will be added later
  public void hangStop(double speed){
    hangMotor.set(0);
  }
  
  
}
