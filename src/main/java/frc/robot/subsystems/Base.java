package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Autoselector;

public class Base extends SubsystemBase {
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;
  private SwerveModule[] modules;

  private AHRS gyro;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Pose2d pose;
  
  private double driveSpeedFactor;
  private double rotSpeedFactor;

  private boolean defenseMode = false;

  SendableChooser<Autoselector.DesiredAuton> autonSendableChooser;
  
  
  public Base() {
    frontLeftModule = new SwerveModule(
      KFrontLeftAngleMotorID,
      KFrontLeftDriveMotorID,
      KFrontLeftMagEncoderID,
      KFrontLeftOffset,
      KFrontLeftDriveReversed,
      KFrontLeftAngleReversed
    );
    frontRightModule = new SwerveModule(
      KFrontRightAngleMotorID,
      KFrontRightDriveMotorID,
      KFrontRightMagEncoderID, 
      KFrontRightOffset,
      KFrontRightDriveReversed,
      KFrontRightAngleReversed
    );
    backLeftModule = new SwerveModule(
      KBackLeftAngleMotorID,
      KBackLeftDriveMotorID,
      KBackLeftMagEncoderID, 
      KBackLeftOffset,
      KBackLeftDriveReversed,
      KBackLeftAngleReversed
    );
    backRightModule = new SwerveModule(
      KBackRightAngleMotorID,
      KBackRightDriveMotorID,
      KBackRightMagEncoderID, 
      KBackRightOffset,
      KBackRightDriveReversed,
      KBackRightAngleReversed
    );
    // modules[0] = frontLeftModule;
    // modules[1] = frontRightModule;
    // modules[2] = backLeftModule;
    // modules[3] = backRightModule;

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    kinematics = new SwerveDriveKinematics(
      KFrontLeftLocation, KFrontRightLocation,
      KBackLeftLocation, KBackRightLocation
    );
    odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
    
    
    driveSpeedFactor = KBaseDriveMidPercent;
    rotSpeedFactor = KBaseRotMidPercent;

    SmartDashboard.putNumber("X and Y PID", 0);
    
    SmartDashboard.putNumber("rot P", 0);

    autonSendableChooser = new SendableChooser<Autoselector.DesiredAuton>();
    autonSendableChooser.addOption("Do Nothing", Autoselector.DesiredAuton.DO_NOTHING);
    autonSendableChooser.addOption("Do Nothing", Autoselector.DesiredAuton.TEST);
    Shuffleboard.getTab("SmartDashboard").add("Auton Name", autonSendableChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double maxDriveSpeedMPS) {
    xSpeed *= maxDriveSpeedMPS;
    ySpeed *= maxDriveSpeedMPS;
    rot *= KMaxAngularSpeed * getRotSpeedFactor();
    
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
    kinematics.toSwerveModuleStates(
      fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())
      : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);
      
      // for (int i = 0; i < states.length; i++) {
        //   modules[i].setDesiredState(states[i]);
        // }
        
        // SmartDashboard.putString("1", ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()).toString());
        
        SmartDashboard.putString("1", ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()).toString());
        if (defenseMode) {
          lockWheels();
        }
    else {
      SmartDashboard.putNumber("frontLeftState", states[0].speedMetersPerSecond);
      SmartDashboard.putNumber("frontRightState", states[1].speedMetersPerSecond);
      SmartDashboard.putNumber("backLeftState", states[2].speedMetersPerSecond);
      SmartDashboard.putNumber("backRightState", states[3].speedMetersPerSecond);
      //setting module states, aka moving the motors
      frontLeftModule.setDesiredState(states[0]);
      frontRightModule.setDesiredState(states[1]);
      backLeftModule.setDesiredState(states[2]);
      backRightModule.setDesiredState(states[3]);
    }
  }

  public void driveAuton(ChassisSpeeds chassisSpeeds) {
    SmartDashboard.putString("1", chassisSpeeds.toString());
    //feeding parameter speeds into toSwerveModuleStates to get an array of SwerveModuleState objects
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(chassisSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, KPhysicalMaxDriveSpeedMPS);
    
    //setting module states, aka moving the motors
    SmartDashboard.putNumber("frontLeftState", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRightState", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeftState", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRightState", states[3].speedMetersPerSecond);

    // System.out.println("SOJAODISJODIAJSOIDJASOIJASOIJDOASIJ");
    System.out.println(Timer.getFPGATimestamp());
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);
  }

  public void lockWheels() {
    frontLeftModule.lockWheel(); 
    frontRightModule.lockWheel();
    backLeftModule.lockWheel();
    backRightModule.lockWheel();
  }
  
  public void resetOdometry(Pose2d pose) {
    resetAllRelEncoders();
    this.pose = pose;
    
    odometry.resetPosition(getHeading(), getPositions(), pose);
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  // recalibrates gyro offset
  public void resetGyro() {
    gyro.reset(); 
    // gyro.setAngleAdjustment(0);
  }

  public void resetGyro(double gyroOffset) {
    gyro.reset();
    gyro.setAngleAdjustment(gyroOffset);
  }

  public void resetAllRelEncoders() {
    frontLeftModule.resetRelEncoders();
    frontRightModule.resetRelEncoders();
    backLeftModule.resetRelEncoders();
    backRightModule.resetRelEncoders();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = frontLeftModule.getPosition();
    positions[1] = frontRightModule.getPosition();
    positions[2] = backLeftModule.getPosition();
    positions[3] = backRightModule.getPosition();

    return positions;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }
  
  public void resetOdometry() {
    resetAllRelEncoders();
    pose = new Pose2d();
    
    odometry.resetPosition(getHeading(), getPositions(), pose);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getHeadingDeg());
  }
  public double getHeadingDeg() {
    return -gyro.getAngle();
    // return -gyro.getFusedHeading();
  }

  public double getRoll() {
    return gyro.getRoll();
  }
  public double getPitch() {
    return gyro.getPitch();
  }
  
  public double getDriveSpeedFactor() {
    return driveSpeedFactor;
  }
  public void setDriveSpeedFactor(double speedFactor) {
    driveSpeedFactor = speedFactor;
  }
  
  public double getRotSpeedFactor() {
    return rotSpeedFactor;
  }
  public void setRotSpeedFactor(double speedFactor) {
    rotSpeedFactor = speedFactor;
  }
  
  public boolean getDefenseMode() {
    return defenseMode;
  }
  public void setDefenseMode(boolean defenseMode) {
    this.defenseMode = defenseMode;
  }

  public Autoselector.DesiredAuton getDesiredAuton() {
    return autonSendableChooser.getSelected();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Fused Heading", gyro.getFusedHeading());
    SmartDashboard.putNumber("Heading", -gyro.getAngle());
    SmartDashboard.putString("odometry pose", odometry.getPoseMeters().toString());
    SmartDashboard.putNumber("BackLeftCanCoderPos", backLeftModule.getMagDegRaw());
    SmartDashboard.putNumber("FrontLeftCanCoderPos", frontLeftModule.getMagDegRaw());
    SmartDashboard.putNumber("BackRightCanCoderPos", backRightModule.getMagDegRaw());
    SmartDashboard.putNumber("FrontRightCanCoderPos", frontRightModule.getMagDegRaw());
    odometry.update(getHeading(), getPositions());
    pose = odometry.getPoseMeters();
  }
}
  
  