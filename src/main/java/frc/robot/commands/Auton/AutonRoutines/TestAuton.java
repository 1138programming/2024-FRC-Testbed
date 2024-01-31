// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.AutonRoutines;

import static frc.robot.Constants.KDriveController;
import static frc.robot.Constants.KMaxAcceleration;
import static frc.robot.Constants.KPhysicalMaxDriveSpeedMPS;
import static frc.robot.Constants.KTrajectoryJson1;
import static frc.robot.Constants.KTrajectoryJson2;
import static frc.robot.Constants.KTrajectoryJson3;
import static frc.robot.Constants.KtrajectoryConfig;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Robot;
import frc.robot.Util.DrivePathFollower;
import frc.robot.Util.TrajGen;
import frc.robot.commands.Auton.RunTrajectory;
import frc.robot.subsystems.Base;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton extends SequentialCommandGroup {
  Trajectory trajectory1 = new Trajectory();
  Trajectory trajectory2 = new Trajectory();
  Trajectory trajectory3 = new Trajectory();
  SwerveControllerCommand path1follow;
  SwerveControllerCommand path2follow;
  SwerveControllerCommand path3follow;

  /** Creates a new TestAuton. */
  public TestAuton(Base base, DrivePathFollower drivePathFollower) {
    try {
      Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(KTrajectoryJson1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
      Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(KTrajectoryJson2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
      Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(KTrajectoryJson3);
      trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
    }
    catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + KTrajectoryJson1, ex.getStackTrace());
    }

    base.resetPose(trajectory1.getInitialPose());
    base.resetGyro();

    path1follow = new SwerveControllerCommand(
      trajectory1, base::getPose, base.getKinematics(), KDriveController, base::setModuleStates, base);

    path2follow = new SwerveControllerCommand(
      trajectory2, base::getPose, base.getKinematics(), KDriveController, base::setModuleStates, base);
     
    path3follow = new SwerveControllerCommand(
      trajectory3, base::getPose, base.getKinematics(), KDriveController, base::setModuleStates, base);
    
      // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new RunTrajectory(base, trajectory, drivePathFollower)
      path1follow,
      new RunCommand(() -> base.resetPose(trajectory2.getInitialPose()), base),
      path2follow
      // path3follow
    );
  }
}
