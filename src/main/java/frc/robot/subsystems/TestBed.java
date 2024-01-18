// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class TestBed extends SubsystemBase {
  private CANSparkMax m1;
  private CANSparkMax m2;
  private CANSparkMax m3;
  private CANSparkMax m4;

  private CANSparkFlex vortex;

  private TalonFX t1;
  private TalonFX t2;
  
  // private DoubleSolenoid s1;
  // private DoubleSolenoid s2;
  // private DoubleSolenoid s3;
  // private DoubleSolenoid s4;
  // private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public TestBed() {
    // m1 = new CANSparkMax(1,MotorType.kBrushless);
    // m2 = new CANSparkMax(2,MotorType.kBrushless);
    // m3 = new CANSparkMax(3,MotorType.kBrushless);
    // m4 = new CANSparkMax(4,MotorType.kBrushless);

    // vortex = new CANSparkFlex(KVortexID, MotorType.kBrushless);
    
    t1 = new TalonFX(KTalon1ID);
    t2 = new TalonFX(KTalon2ID);
   
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
    // This method will be called once per scheduler run
  }

  public void moveSpark(CANSparkMax canSparkMaxID, double speed) {
    canSparkMaxID.set(speed);
  }
  public void moveTalon(TalonFX t, double speed)
  {
    t.set(speed);
  }
  // public void moveVortex(double speed) {
  //   vortex.set(speed);
  // }

  // public void movepiston () {
  //   System.out.println( s1.get());
  //   s1.toggle();
  // }
  
}
