// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import static frc.robot.Constants.KFlywheelspinfast;
import static frc.robot.Constants.Kfastspinvelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class Spinflywheelfast extends CommandBase {
  /** Creates a new Spinflywheelfast. */
  
  private Flywheel flywheel;

  public Spinflywheelfast(Flywheel flywheel) {
    this.flywheel = flywheel;
    addRequirements(flywheel);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setFlywheelSpeed(KFlywheelspinfast, Kfastspinvelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.spinFlywheelToSpeed();
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