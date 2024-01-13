package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.hardware.DeviceIdentifier;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.CANSparkMaxLowLevel.FollowConfig.Config;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  // magEncoder = absolute encoder to reset position of relative angle encoders
  private CANcoder canCoder;
  

  // Relative encoders are used for robot odometry and controlling speed/position
  private RelativeEncoder driveEncoder;

  private PIDController angleController;

  private double offset;

  private SimpleMotorFeedforward feedforward;
  private PIDController driveController;

  public SwerveModule(int angleMotorID, int driveMotorID, int encoderPort, double offset, 
                      boolean driveMotorReversed, boolean angleMotorReversed) {
    angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    this.angleMotor.setInverted(angleMotorReversed);
    this.driveMotor.setInverted(driveMotorReversed);
    
    this.driveMotor.setSmartCurrentLimit(KDriveMotorCurrentLimit);
    this.angleMotor.setSmartCurrentLimit(KAngleMotorCurrentLimit);

    
    canCoder = new CANcoder(encoderPort);

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = offset; // CHANGE TO ROTATIONS
    canCoder.getConfigurator().apply(canCoderConfig);

    driveEncoder = driveMotor.getEncoder();
    
    driveEncoder.setPositionConversionFactor(KDriveMotorRotToMeter);
    driveEncoder.setVelocityConversionFactor(KDriveMotorRPMToMetersPerSec);
    
    angleController = new PIDController(KAngleP, KAngleI, KAngleD);
    angleController.enableContinuousInput(-180, 180); // Tells PIDController that 180 deg is same in both directions

    driveController = new PIDController(0.64442, 0, 0); //Reset These Values

    // SmartDashboard.putNumber("Wheel angle PID", 0);
  }
  
  
  public void setDesiredState(SwerveModuleState desiredState) {
    double angleMotorOutput;
    double driveMotorOutput;
    
    Rotation2d currentAngleR2D = getAngleR2D();
    desiredState = SwerveModuleState.optimize(desiredState, currentAngleR2D);
    angleMotorOutput = angleController.calculate(getMagDegRaw(), desiredState.angle.getDegrees());
    
    driveMotorOutput = desiredState.speedMetersPerSecond / KPhysicalMaxDriveSpeedMPS;
    
    angleMotor.set(angleMotorOutput);
    driveMotor.set(driveMotorOutput);    
  }
  
  public void lockWheel() {
    double angleMotorOutput;
    if (angleMotor.getDeviceId() == KFrontLeftAngleMotorID || angleMotor.getDeviceId() == KBackRightAngleMotorID) {
      angleMotorOutput = angleController.calculate(getMagDegRaw(), 45);
    }
    else {
      angleMotorOutput = angleController.calculate(getMagDegRaw(), -45);
    }
    
    angleMotor.set(angleMotorOutput);
    driveMotor.set(0);
  }
  
  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition(getDriveEncoderPos(), getAngleR2D());
    return position;
  }
  
  public void stop() {
    driveMotor.set(0);
    angleMotor.set(0);
  }
  
  public void resetRelEncoders() {
    driveEncoder.setPosition(0);
  }
  
  // Drive Encoder getters
  public double getDriveEncoderPos() {
    return driveEncoder.getPosition();
  }
  public double getDriveEncoderVel() {
    return driveEncoder.getVelocity();
  }
  
  // Angle Encoder getters
  public double getMagDegRaw() {
    double pos = canCoder.getAbsolutePosition().getValueAsDouble() * 360;
    return pos;
  }

  public Rotation2d getAngleR2D() {
    return Rotation2d.fromDegrees(getMagDegRaw()); 
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean(getName(), KBackLeftAngleReversed)
    // angleController.setP(SmartDashboard.getNumber("Wheel angle PID", 0));
  }
}
