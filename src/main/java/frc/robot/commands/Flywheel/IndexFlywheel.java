// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;


public class IndexFlywheel extends CommandBase {
private Flywheel flywheel;

  /** Creates a new IndexFlywheel. */
  public IndexFlywheel(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (flywheel.getSpinEncoderVelocity() >= flywheel.getTargetVelocity()) {
    flywheel.SpinFlywheelIndexer(0.9);
  }
  else{
    flywheel.SpinFlywheelIndexer(0.0);
  }

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
