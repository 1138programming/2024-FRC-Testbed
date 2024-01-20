// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FlywheelTesting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkMax;

public class SpinInFlywheel extends Command {
  private final SparkMax sparkMax;
  /** Creates a new SpinoutFlywheel. */
  public SpinInFlywheel(SparkMax sparkMax) {
    this.sparkMax = sparkMax;
    addRequirements(sparkMax);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sparkMax.moveBothMotors(-0.9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sparkMax.moveBothMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
