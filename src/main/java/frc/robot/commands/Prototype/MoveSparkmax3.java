// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Prototype;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sparkmax3;

public class MoveSparkmax3 extends CommandBase {
  private Sparkmax3 sparkmax3;
  /** Creates a new MoveSparkmax3. */
  public MoveSparkmax3(Sparkmax3 sparkmax3) {
    this.sparkmax3 = sparkmax3;
    addRequirements(sparkmax3);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sparkmax3.movemotor(0.5);
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
