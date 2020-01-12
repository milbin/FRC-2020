/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class TestTrajectory {

    public static Trajectory generate() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10);
    
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // figure out how to get the configuration to the robot container
        var startPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
        var endPos = new Pose2d(Units.feetToMeters(-5), Units.feetToMeters(40), Rotation2d.fromDegrees(0.0));
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(-5), Units.feetToMeters(10)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(-5), Units.feetToMeters(20)));
        return TrajectoryGenerator.generateTrajectory(startPos, interiorWaypoints, endPos, config);
    }
}
