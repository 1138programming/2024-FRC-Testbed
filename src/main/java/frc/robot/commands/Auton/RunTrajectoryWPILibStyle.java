// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import static frc.robot.Constants.KDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Base;

public class RunTrajectoryWPILibStyle extends Command {
  private final Base base;
  private Trajectory trajectory;
  private SwerveControllerCommand swerveControllerCommand;
  private HolonomicDriveController driveController;

  /** Creates a new RunTrajectoryWPILibStyle. */
  public RunTrajectoryWPILibStyle(Base base, Trajectory trajectory) {
    this.base = base;
    this.trajectory = trajectory;

    swerveControllerCommand = new SwerveControllerCommand(
      trajectory, base::getPose, base.getKinematics(), KDriveController, base::setModuleStates, base);

    addRequirements(base);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
