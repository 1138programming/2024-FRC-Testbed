// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.IdleMode;


public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  
  private CANSparkMax vortexSparkFlex;

  private RelativeEncoder vortexSpin;

  private CANCoder flywheeelCanCoder;
  
  public Flywheel() {

    vortexSparkFlex = new CANSparkMax(KFlyWheelMotorID, MotorType.kBrushless);

    flywheeelCanCoder = new CANCoder(KFlywheelEncoderID);

    vortexSparkFlex.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void SpinFlywheel(double speed){
  vortexSparkFlex.set(speed);
}
}
