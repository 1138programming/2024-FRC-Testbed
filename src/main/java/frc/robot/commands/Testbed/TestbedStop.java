// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Testbed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Testbed;

public class TestbedStop extends Command {
  private Testbed testbed;
  /** Creates a new TestbedStop. */
  public TestbedStop(Testbed testbed) {
    this.testbed = testbed;
    addRequirements(testbed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // testbed.moveSpark(0, 0);
    // testbed.moveSpark(1, 0);
    // testbed.moveSpark(2, 0);
    // testbed.moveSpark(3, 0);

    // testbed.moveTalon(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
