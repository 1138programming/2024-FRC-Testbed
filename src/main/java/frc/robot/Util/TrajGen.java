package frc.robot.Util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class TrajGen {
        public static Trajectory generateTrajectoryFromFile(String file_path, TrajectoryConfig config) {
        try {
            Path traj_path = Filesystem.getDeployDirectory().toPath().resolve(file_path);
            TrajectoryGenerator.ControlVectorList control_vectors = WaypointReader.getControlVectors(traj_path);
            SmartDashboard.putString("control vectors", control_vectors.toString());
            
            return TrajectoryGenerator.generateTrajectory(control_vectors, config);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + file_path, ex.getStackTrace());
            return null;
        } 
}
}
