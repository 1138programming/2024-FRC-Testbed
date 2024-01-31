// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolea
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  // Sensing - CANBUS -------------------------------------------------------------
  
  // BASE - Values will be changed for CANCoders (which will not use DIO)
  public static final int KFrontLeftMagEncoderID = 1;
  public static final int KFrontRightMagEncoderID = 2;
  public static final int KBackLeftMagEncoderID = 3;
  public static final int KBackRightMagEncoderID = 4;
  
  // End of Sensing - CANBUS **************************************************
  
  
  // Motor IDs by Subsystem ------------------------------------------------------
  
  // Base
  public static final int KFrontLeftAngleMotorID = 1;  	// SparkMax + NEO
  public static final int KFrontLeftDriveMotorID = 2;  	// SparkMax + NEO
  
  public static final int KFrontRightAngleMotorID = 3;  	// SparkMax + NEO
  public static final int KFrontRightDriveMotorID = 4;  	// SparkMax + NEO
  
  public static final int KBackLeftAngleMotorID = 5;  	  // SparkMax + NEO
  public static final int KBackLeftDriveMotorID = 6;  	  // SparkMax + NEO
  
  public static final int KBackRightAngleMotorID = 7;  	// SparkMax + NEO
  public static final int KBackRightDriveMotorID = 8;  	// SparkMax + NEO

  // Testbed

  public static final int KTalon1ID = 9;
  public static final int KTalon2ID = 10;
  
  public static final int KSpark1ID = 11;
  public static final int KSpark2ID = 12;
  public static final int KSpark3ID = 13;
  public static final int KSpark4ID = 14;

  public static final int KVortex1ID = 18;
  public static final int KVortex2ID = 15;
  
  // End of Motor Section *****************************************************
  
  // CURRENT LIMITS
  public static final int KDriveMotorCurrentLimit = 40;
  public static final int KAngleMotorCurrentLimit = 30;
  
  // Subsystem Constants -----------------------------------------------------

  // Swerve Modules
  public static final double KAngleP = 0.006;
  public static final double KAngleI = 0;
  public static final double KAngleD = 0;
  
  public static final double KDriveP = 0.2;
  public static final double KDriveI = 0.75;
  public static final double KDriveD = 0.005;
  
  // used for math in Constants and SwerveModules to set up encoder units
  public static final double KDegPerRotation = 360;
  public static final double KNeoMaxRPM = 5700;
  
  // Sets up drive encoders to use meters as the unit

  // FIND VALUES FOR TESTBED
  private static final double KDriveMotorGearRatio = 1/6.75;
  private static final double KWheelDiameterMeters = 0.1016;
  public static final double KDriveMotorRotToMeter = KDriveMotorGearRatio * KWheelDiameterMeters * Math.PI;
  
  public static final double KDriveMotorRPMToMetersPerSec = KDriveMotorRotToMeter / 60;
  
  public static final double KAngleGearRatio = 21.42857; // 150/7 : 1
  
  public static final double KPhysicalMaxDriveSpeedMPS = KNeoMaxRPM * KDriveMotorRPMToMetersPerSec;
  public static final double KMaxAcceleration = 4;

  // find for testbed
  public static final double KMaxAngularSpeed = 3.5; 
  
    // Low and high percent: sets max speed of drivetrain for driver
  public static final double KBaseDriveLowPercent = 0.25;
  public static final double KBaseDriveMidPercent = 0.5;
  public static final double KBaseDriveMaxPercent = 1;

  public static final double KBaseRotLowPercent = 0.75;
  public static final double KBaseRotMidPercent = 1;
  public static final double KBaseRotMaxPercent = 1.5;
  
  // Offsets for absolute encoders, used to set up angle encoders

  // double check
  public static final double KFrontLeftOffset = 82;
  public static final double KFrontRightOffset = -33.48;  
  public static final double KBackLeftOffset =  -22.23;
  public static final double KBackRightOffset = -8.701;
  // public static final double KFrontLeftOffset = 0;
  // public static final double KFrontRightOffset = 0;  
  // public static final double KBackLeftOffset =  0;
  // public static final double KBackRightOffset = 0;


  // public static final double KFrontLeftOffset = -0.90045464122; //-315.263671 deg
  // public static final double KFrontRightOffset = -0.21484375; //-77.34375 
  // public static final double KBackLeftOffset = -0.0810546875; //-29.1796875 deg
  // public static final double KBackRightOffset = -0.0627441389; //-22.587890 deg 
  // public static final double KFrontLeftOffset = -0.875732419; //-315.263671 deg
  // 0.02472222222 
  
    // Describes the locations of the swerve modules relative to the center of the robot
  // Important for kinematics
  public static final double KWheelDistanceFromCenter = 0.3048;
  public static final Translation2d KFrontLeftLocation = new Translation2d(
    KWheelDistanceFromCenter, KWheelDistanceFromCenter
  );
  public static final Translation2d KFrontRightLocation = new Translation2d(
    KWheelDistanceFromCenter, -KWheelDistanceFromCenter
  );
  public static final Translation2d KBackLeftLocation = new Translation2d(
    -KWheelDistanceFromCenter, KWheelDistanceFromCenter
  );
  public static final Translation2d KBackRightLocation = new Translation2d(
    -KWheelDistanceFromCenter, -KWheelDistanceFromCenter
  );
  public static final double KWheelRadialDistanceFromCenter = 0.43105;

    // Setting up which motors are reversed
  public static final boolean KFrontLeftDriveReversed = false;
  public static final boolean KFrontRightDriveReversed = false;
  public static final boolean KBackLeftDriveReversed = false;
  public static final boolean KBackRightDriveReversed = false;
  
  public static final boolean KFrontLeftAngleReversed = true;
  public static final boolean KFrontRightAngleReversed = true;
  public static final boolean KBackLeftAngleReversed = true;
  public static final boolean KBackRightAngleReversed = true;
  
  public static final boolean KFrontLeftDriveEncoderReversed = false;
  public static final boolean KFrontRightDriveEncoderReversed = false;
  public static final boolean KBackLeftDriveEncoderReversed = false;
  public static final boolean KBackRightDriveEncoderReversed = false;

  public static final double KGyroOffset = 180;
 
  
  //Limelight
  public static final double KLimelightHeight = 19.5; // inches
  public static final double KMidPoleHeight = 25; // inches
  // public static final double KHighPoleHeight = 0; // inches
  public static final double KHeightDifference = KMidPoleHeight - KLimelightHeight; // inches
  public static final double KLimelightAngle = -5;
  public static final double KLimelightRange = 29.8;
  public static final double kDistanceWhenNoTarget = 0;
  public static final double kHorizDistanceWhenNoTarget = 0;
  public static final double kDesiredYOffset = 1;
  public static final double kDesiredXOffset = 1;
  public static final double kLimelightXOffsetDeadzone = 0.05;
  public static final double KDistanceOffset = 0;
  public static final double KHorizDistanceOffset = 0;
  public static final double KGoalWidth = 15;
  
  public static final int KAprilTagPipeline = 0;
  public static final int KReflectiveTapePipeline = 1;
  
  public static final double KDistanceMoveOffset = 1;
  
  public static final double KOffsetFromAprilTag = 1;
  
  public static final double[] KXCoordinateOfTag = { 
    7.24310,
    7.24310,
    7.24310,
    7.90832,
    -7.90832,
    -7.24310,
    -7.24310,
    -7.24310,
  };
  
  public static final double[] KYCoordinateOfTag = { 
    -2.93659,
    -1.26019,
    0.41621,
    2.74161,
    2.74161,
    0.41621,
    -1.26019,
    -2.93659
  };
  
  public static final double KLimelightRotateP = 0.01;
  public static final double KLimelightRotateI = 0;
  public static final double KLimelightRotateD = 0;
  public static final double KLimelightMoveDeadzone = 0.01;
  
  
  public static final double KTagLimelightMoveP = 0.08;
  public static final double KTagLimelightMoveI = 0;
  public static final double KTagLimelightMoveD = 0;
  
  public static final double KTapeLimelightMoveP = 0.005;
  public static final double KTapeLimelightMoveI = 0;
  public static final double KTapeLimelightMoveD = 0;
  
  public static final double KForwardControllerP = 1;
  public static final double KForwardControllerI = 0;
  public static final double KForwardControllerD = 0;

  public static final double KStrafeControllerP = 1;
  public static final double KStrafeControllerI = 0;
  public static final double KStrafeControllerD = 0;
  
  public static final double KRotControllerP = 0;
  public static final double KRotControllerI = 0;
  public static final double KRotControllerD = 0;

  public static final double KRotMaxVelocity = 2 * Math.PI;
  public static final double KRotMaxAcceleration = 2 * Math.PI * KRotMaxVelocity;

  public static final TrapezoidProfile.Constraints KRotControllerConstraints = new TrapezoidProfile.Constraints(
                                KRotMaxVelocity, KRotMaxAcceleration);

  public static final PIDController KForwardController = new PIDController(KForwardControllerP, KForwardControllerI, KForwardControllerD);
  public static final PIDController KStrafeController = new PIDController(KStrafeControllerP, KStrafeControllerI, KStrafeControllerD);
  public static final ProfiledPIDController KRotationController = new ProfiledPIDController(KRotControllerP, KRotControllerI, KRotControllerD, KRotControllerConstraints);
  public static final HolonomicDriveController KDriveController = new HolonomicDriveController(KForwardController, KStrafeController, KRotationController);


  // public static final double KRotMaxVelocity = Math.PI;
  // public static final double KRotMaxAcceleration = Math.PI * KRotMaxVelocity;
  
  
  // need to find for testbed
  // public static final double ks = 0.20309;
  // public static final double kv = 2.5574;
  // public static final double ka = 0.38422;


  // public static final double kFieldLength = Units.inchesToMeters(651.25);
  public static final double kFieldLength = Units.inchesToMeters(653.2);
  public static final double kFieldWidth = Units.inchesToMeters(323.28);
  // public static final double kFieldWidth = Units.inchesToMeters(315.5);

  public static final String KTrajectoryJson1 = "paths/Path1.wpilib.json";
  public static final String KTrajectoryJson2 = "paths/Path2.wpilib.json";
  public static final String KTrajectoryJson3 = "paths/Path3.wpilib.json";

  public static final TrajectoryConfig KtrajectoryConfig = new TrajectoryConfig(KPhysicalMaxDriveSpeedMPS,KMaxAcceleration);

  public static final HolonomicPathFollowerConfig KPathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      KPhysicalMaxDriveSpeedMPS, 
      KWheelRadialDistanceFromCenter, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  
  // public static final PathConstraints KPathPlannerConstraints = 
  //   new PathConstraints(4, 3, 1, 1);

  
  // try {
  //   private static final Trajectory KPath1 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(KTrajectoryJson1));
  // } catch (IOException ex) {
  //     DriverStation.reportError("Unable to open trajectory: " + t, ex.getStackTrace());
  //  }
}

