// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Testbed;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
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
    testbed.moveVortex(0.5);
    // testbed.moveVortex(1, -Robot.m_robotContainer.getXboxRightYAxis());
    // SmartDashboard.putNumber("joystick", Robot.m_robotContainer.getXboxRightYAxis());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // testbed.moveVortex(0, 0);
    // testbed.moveVortex(1, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
