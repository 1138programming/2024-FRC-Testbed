// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import static frc.robot.Constants.KMaxAcceleration;
import static frc.robot.Constants.KMaxAngularSpeed;
import static frc.robot.Constants.KPhysicalMaxDriveSpeedMPS;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.DrivePathFollower;
import frc.robot.subsystems.Base;

public class RunTrajectory extends Command {
  private Trajectory t;
  private DrivePathFollower drivePathFollower;
  private final Base base;

  /** Creates a new RunTrajectory. */
  public RunTrajectory(Base base, Trajectory t, DrivePathFollower drivePathFollower) {
    this.base = base;
    this.t = t;
    this.drivePathFollower = drivePathFollower;
   }
    // Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DrivePathFollower.createConfig(KPhysicalMaxDriveSpeedMPS, KMaxAcceleration, 0, 0);
    base.resetOdometry(t.getInitialPose());
    drivePathFollower.setTrajectory(t, base.getHeading(), base.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putString("CHASSIS SPEEDS", drivePathFollower.update(base.getPose(), Timer.getFPGATimestamp()).toString());
    base.driveAuton(drivePathFollower.update(base.getPose(), Timer.getFPGATimestamp()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePathFollower.isFinished();
  }
}
