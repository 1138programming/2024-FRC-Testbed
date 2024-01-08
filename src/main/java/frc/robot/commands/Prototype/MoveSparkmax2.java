// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Prototype;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sparkmax2;

public class MoveSparkmax2 extends CommandBase {
  private Sparkmax2 sparkmax2;
  /** Creates a new MoveSparkmax2. */
  public MoveSparkmax2(Sparkmax2 sparkmax2) {
    this.sparkmax2 = sparkmax2;
    addRequirements(sparkmax2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sparkmax2.movemotor(0.5);
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
