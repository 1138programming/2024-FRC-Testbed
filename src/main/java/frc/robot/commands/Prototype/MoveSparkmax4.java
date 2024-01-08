// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Prototype;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sparkmax4;

public class MoveSparkmax4 extends CommandBase {
  private Sparkmax4 sparkmax4;
  /** Creates a new MoveSparkmax4. */
  public MoveSparkmax4(Sparkmax4 sparkmax4) {
    this.sparkmax4 = sparkmax4;
    addRequirements(sparkmax4);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sparkmax4.movemotor(0.5);
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
