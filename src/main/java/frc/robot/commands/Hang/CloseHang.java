// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
import frc.robot.Constants;

public class CloseHang extends CommandBase {
  /** Creates a new ShortenHang. */
  public Hang hang;
  private double speed;

  public CloseHang(Hang hang, double speed) {
    this.hang = hang;
    this.speed = speed;
    addRequirements(hang);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //hang.getHangPosition(KHang)
    hang.moveHang(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.hangStop();
    //hang.setHangPosDown(null);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
