// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.positioning;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.HolonomicController;
import frc.robot.Constants.Drive.HolonomicController.ThetaController;
import frc.robot.Constants.Drive.HolonomicController.ThetaController.Constraints;



import java.util.ArrayList;

/**
 * Static utility class for generating WPILib {@link Trajectory Trajectories} and more
 */
public class Team937TrajectoryUtils {

  /**
   * Static method for generating WPILib {@link Trajectory Trajectories} from A* {@link Path Paths}.
   *
   * @param path The Path to generate a Trajectory from
   * @return The generated Trajectory
   */
  public static Trajectory generateTrajectory(Path path) {
    ArrayList<Double[]> pathList = path.getPathList();

    /* stealing max accel and velocity from holonomicController */
    TrajectoryConfig config =
        new TrajectoryConfig(
            Constants.Drive.HolonomicController.ThetaController.Constraints.MAX_VELOCITY,
            Constants.Drive.HolonomicController.ThetaController.Constraints.MAX_ACCELERATION);

    /* I *THINK* that we don't need to care about rotation because the HolonomicController calls will do that separately */
    Pose2d start = new Pose2d(pathList.get(0)[0], pathList.get(0)[1], Rotation2d.fromDegrees(0));
    Pose2d end =
        new Pose2d(
            pathList.get(pathList.size())[0],
            pathList.get(pathList.size())[1],
            Rotation2d.fromDegrees(0));

    ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
    for (Double[] coordinate : pathList) {
      waypoints.add(new Translation2d(coordinate[0], coordinate[1]));
    }

    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
  }

  /**
   * Generates {@link Trajectory} out of Json File intended for pathweaver.
   * @param jsonFile - A filepath to a JSON file
   * @return A {@link Trajectory}
   */
  public static Trajectory generateTrajectory(String jsonFile) {

    try {
      return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(jsonFile));
    }
    catch (java.io.IOException e) {
      DriverStation.reportError("Unable to open trajectory JSON file: " + jsonFile, e.getStackTrace());
      return null;
    }
  }

}
