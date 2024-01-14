// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Testbed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Testbed;

public class MoveSparkmaxWithJoystick extends Command {
  private Testbed testbed;
  private int sparkNum;
  /** Creates a new MoveSparkmax. */
  public MoveSparkmaxWithJoystick(Testbed testbed, int sparkNum) {
    this.testbed = testbed;
    this.sparkNum = sparkNum;
  
    addRequirements(testbed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testbed.moveSpark(sparkNum, Robot.m_robotContainer.getXboxLeftYAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testbed.moveSpark(sparkNum, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
