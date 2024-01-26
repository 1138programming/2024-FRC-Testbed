// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
//import static frc.robot.Constants.kHangEncoderValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.ctre.phoenix.sensors.CANCoder;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.KHangBottomLS;
import static frc.robot.Constants.KHangMotorID;
import static frc.robot.Constants.KHangTopLS;

//import com.ctre.phoenix.motorcontrol.ControlMode;

//import edu.wpi.first.wpilibj.CAN;
// add a limit switch
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hang extends SubsystemBase {
  //Hang Motor
  private CANSparkMax hangMotor;
  // Hang Limit Switchs
  private DigitalInput hangTopLS;
  private DigitalInput hangBottomLS;
  
  public Hang() {
    //Motor
    hangMotor = new CANSparkMax(KHangMotorID, MotorType.kBrushless);

    hangTopLS = new DigitalInput(KHangTopLS);
    hangBottomLS = new DigitalInput(KHangBottomLS);
    }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  

  public double getHangPosition(){
    return hangMotor.getEncoder().getPosition();
  }
  //Do I need these two??
   public void setHangPosUp(){
    while (hangMotor.getEncoder().getPosition() <= 0 && !getHangTopLS()){
      hangMotor.getEncoder().setPosition(20); 
    }
  }

  //Create constants for set positions
  public void setHangPosDown(){
    while (hangMotor.getEncoder().getPosition() >= 0 && !getHangBottomLS()){
      hangMotor.getEncoder().setPosition(-20);
    }  }

  public void moveHang (double speed){
    hangMotor.set(speed);
    if(speed>0){
      if (getHangTopLS()){
        hangMotor.set(0);
      }
    }
    else if(speed<0) {
      if (getHangBottomLS()){
        hangMotor.set(0);
      }
    }
  }


  public boolean getHangTopLS(){
    return hangTopLS.get();
  }

  public boolean getHangBottomLS(){
    return hangBottomLS.get();
  }

  //Limit Switches will be added later
  public void hangStop(){
    hangMotor.set(0);
  }
  
  
}
