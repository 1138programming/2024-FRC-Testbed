// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Testbed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Testbed;

public class MoveVortex extends Command {
  private Testbed testbed;

  /** Creates a new moveVortexWithJoystick. */
  public MoveVortex(Testbed testbed) {
    this.testbed = testbed;
    addRequirements(testbed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // testbed.moveVortex(0.9);
    testbed.moveVortex(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testbed.stopVortex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
