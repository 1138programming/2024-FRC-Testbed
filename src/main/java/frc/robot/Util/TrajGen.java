package frc.robot.Util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public final class TrajGen {
        public static Trajectory generateTrajectoryFromFile(String file_path, TrajectoryConfig config, boolean isRed) {
        try {
            Path traj_path = Filesystem.getDeployDirectory().toPath().resolve(file_path);
            TrajectoryGenerator.ControlVectorList control_vectors = WaypointReader.getControlVectors(traj_path, isRed);
            
            return TrajectoryGenerator.generateTrajectory(control_vectors, config);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + file_path, ex.getStackTrace());
            return null;
        } 
}
}
