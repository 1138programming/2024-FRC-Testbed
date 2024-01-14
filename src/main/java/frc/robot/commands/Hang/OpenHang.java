// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

// import static frc.robot.Constants.kHangEncoderValue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hang;

public class OpenHang extends CommandBase {
  /** Creates a new OpenHang. */
  private Hang hang;
  public OpenHang(Hang hang) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hang = hang;
    addRequirements(hang);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hang.openHang(0.5);
    
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
