// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Testbed extends SubsystemBase {
  private CANSparkMax m1;
  private CANSparkMax m2;
  private CANSparkMax []sparkMaxes;
  private TalonSRX []TalonSRXs;
  private CANSparkFlex []sparkFlexes;

  private CANSparkFlex vortex1;
  private CANSparkFlex vortex2;

  private TalonSRX t1;
  private TalonSRX t2;
  private RelativeEncoder m1_encoder;
  private RelativeEncoder m2_encoder;
  private RelativeEncoder v1_encoder;
  private RelativeEncoder v2_encoder;
  
  // private DoubleSolenoid s1;
  // private DoubleSolenoid s2;
  // private DoubleSolenoid s3;
  // private DoubleSolenoid s4;
  // private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public Testbed() {
    //SparkMax
    m1 = new CANSparkMax(KSpark3ID, MotorType.kBrushless);
    m2 = new CANSparkMax(KSpark4ID, MotorType.kBrushless);
    sparkMaxes = new CANSparkMax[4];
    sparkMaxes[0] = m1;
    sparkMaxes[1] = m2;

    //SparkFlex
    vortex1 = new CANSparkFlex(KVortex1ID, MotorType.kBrushless);
    vortex2 = new CANSparkFlex(KVortex2ID, MotorType.kBrushless);
    sparkFlexes = new CANSparkFlex[2];
    sparkFlexes[0] = vortex1;
    sparkFlexes[1] = vortex2;

    //TalonFX
    t1 = new TalonSRX(KTalon1ID);
    t2 = new TalonSRX(KTalon2ID);
    TalonSRXs = new TalonSRX[2];
    TalonSRXs[0] = t1;
    TalonSRXs[1] = t2;


    m1_encoder = m1.getEncoder();
    m2_encoder = m2.getEncoder();
    v1_encoder = vortex1.getEncoder();
    v2_encoder = vortex2.getEncoder();
    
    // s1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 4);
    // s1.set(DoubleSolenoid.Value.kReverse);
    // s2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 5);
    // s2.set(DoubleSolenoid.Value.kReverse);
    // s3 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 6);
    // s3.set(DoubleSolenoid.Value.kReverse);
    // s4 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 7);
    // s4.set(DoubleSolenoid.Value.kOff);
  }

  @Override
  public void periodic() {
  
    SmartDashboard.putNumber("vortexspeed", v1_encoder.getVelocity());
    SmartDashboard.putNumber("m1 speed", m1_encoder.getVelocity());
    SmartDashboard.putNumber("m2 speed", m2_encoder.getVelocity());
  }

  public void moveSpark1(double speed) {
    m1.set(speed);
  }
  public void moveSpark2(double speed) {
    m2.set(speed);
  }
  public void moveBothSparks(double speed) {
    m1.set(speed);
    m2.set(speed);
  }

  // public void stopSparkIfOverVolted(int MotorNum) {
  //   sparkMaxes[MotorNum].getvol
  // }

  public void moveTalon(int t, double speed)
  {
    TalonSRXs[t].set(ControlMode.PercentOutput, speed);
  }

  public void moveVortex(double speed) {
    vortex1.set(speed);
  }
  public void setVortexVelocity(int speed) {
    vortex1.setVoltage(speed);
  }
  
  public void stopVortex() {
    vortex1.stopMotor();
  }
  public void stopNeo() {
    m1.stopMotor();
    m2.stopMotor();
    m1.stopMotor();
    m2.stopMotor();
  }
  public void stopTalon() {
    t1.set(ControlMode.PercentOutput, 0);
    t2.set(ControlMode.PercentOutput, 0);
  }
  

  // public void movepiston () {
  //   System.out.println( s1.get());
  //   s1.toggle();
  // }
  
}
