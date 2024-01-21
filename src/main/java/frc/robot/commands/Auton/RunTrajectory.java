// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;

public class RunTrajectory extends Command {
  private String path;
  private Trajectory t;
  /** Creates a new RunTrajectory. */
  public RunTrajectory(String path) {
    this.path = path;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      t = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + t, ex.getStackTrace());
   }
  }

   public RunTrajectory(Trajectory t) {
    this.t = t;
   }
    // Use addRequirements() here to declare subsystem dependencies.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
