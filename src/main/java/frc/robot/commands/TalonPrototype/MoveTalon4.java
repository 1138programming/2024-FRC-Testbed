// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TalonPrototype;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Talon4;

public class MoveTalon4 extends CommandBase {
  private Talon4 talon4;
  /** Creates a new MoveTalon1. */
  public MoveTalon4(Talon4 talon4) {
    this.talon4 = talon4;
    addRequirements(talon4);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talon4.moveMotor(0.5);
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
