package frc.robot.Util; 

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;
import frc.robot.Robot;

// thanks 1678
public class DrivePathFollower {
    private final PIDController forwardController;
    private final PIDController strafeController;
    private final ProfiledPIDController rotationController;

    private final HolonomicDriveController driveController;

    private Trajectory currentTrajectory;
    private Rotation2d targetRotation;
    private Double startTime = Double.NaN;

    private boolean isFinished = false;
    
    // Static factory for creating trajectory configs
    public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed, double endSpeed) {
        TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
        config.setStartVelocity(startSpeed);
        config.setEndVelocity(endSpeed);
        config.addConstraint(new CentripetalAccelerationConstraint(10.0));
        return config;
    }

    public DrivePathFollower() {
        forwardController = new PIDController(KForwardControllerP, KForwardControllerI, KForwardControllerD);
        strafeController = new PIDController(KStrafeControllerP, KStrafeControllerI, KStrafeControllerD);
        rotationController = new ProfiledPIDController(KRotControllerP, KRotControllerI, KRotControllerD, KRotControllerConstraints);
        
        rotationController.enableContinuousInput(0, 2 * Math.PI);

        driveController = new HolonomicDriveController(forwardController, strafeController, rotationController);

        SmartDashboard.putNumber("Desired traj speed", 0.0);
    }

    public void setTrajectory(Trajectory trajectory, Rotation2d heading, Pose2d current_pose) {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset(current_pose.getRotation().getRadians());
        startTime = Double.NaN;
        currentTrajectory = trajectory;
        isFinished = false;
        setTargetHeading(heading);
    }

    public void setTargetHeading(Rotation2d newHeading) {
        targetRotation = newHeading;
        if (Robot.flip_trajectories) {
            targetRotation = Rotation2d.fromDegrees(180.0).rotateBy(targetRotation.unaryMinus());
        }
    }

    public Trajectory generateTrajectory(TrajectoryConfig config, Pose2d... poses) {
        ArrayList<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i = 1; i < poses.length - 1; i++) {
            interiorPoints.add(poses[i].getTranslation());
        }
        return TrajectoryGenerator.generateTrajectory(poses[0], interiorPoints, poses[poses.length - 1], config);
    }

    public ChassisSpeeds update(Pose2d current_state, double timestamp) {
        if (startTime.isNaN()) {
            startTime = Timer.getFPGATimestamp();
        }

        if (timestamp > startTime + currentTrajectory.getTotalTimeSeconds()) {
            System.out.println("NULL");
            isFinished = true;
        }

        if (currentTrajectory == null) {
            // System.out.println("NULL");
            return new ChassisSpeeds();
        }
        // 
        Trajectory.State desired_state = currentTrajectory.sample(timestamp - startTime);
        SmartDashboard.putString("Desired state", desired_state.toString());

        SmartDashboard.putNumber("Desired traj speed", desired_state.velocityMetersPerSecond);
        // SmartDashboard.

        return driveController.calculate(current_state, desired_state, targetRotation);
    }

    public boolean isFinished() {
        return currentTrajectory != null && isFinished;
    }
}
