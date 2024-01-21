// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.AutonRoutines;

import static frc.robot.Constants.KTrajectoryJson1;
import static frc.robot.Constants.KtrajectoryConfig;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Util.TrajGen;
import frc.robot.commands.Auton.RunTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton extends SequentialCommandGroup {
  Trajectory traj1 = TrajGen.generateTrajectoryFromFile(KTrajectoryJson1, KtrajectoryConfig);
  /** Creates a new TestAuton. */
  public TestAuton() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunTrajectory(traj1)
    );
  }
}
