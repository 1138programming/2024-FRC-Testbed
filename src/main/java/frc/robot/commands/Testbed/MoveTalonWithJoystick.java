// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Testbed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Testbed;

public class MoveTalonWithJoystick extends Command {
  private Testbed testbed;
  private int talonNum;
  /** Creates a new MoveSparkmax. */
  public MoveTalonWithJoystick(Testbed testbed, int talonNum) {
    this.testbed = testbed;
    this.talonNum = talonNum;
  
    addRequirements(testbed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testbed.moveTalon(talonNum, Robot.m_robotContainer.getXboxLeftYAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testbed.moveTalon(talonNum, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}