package frc.robot.subsystems;

import static frc.robot.Constants.TrajectoryPathnames.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

public class Trajectories {
    private Trajectory pathARed;
    private Trajectory pathBRed;
    private Trajectory pathABlue;
    private Trajectory pathBBlue;
    private Trajectory barrelRacingPath;
    private Trajectory bouncePath;
    private Trajectory slalomPath;
    private Trajectory pathweaverTestPath; // In case we need to make a test path
    private TrajectoryConfig config;
    private TrajectoryConfig specialConfig;

    // We don't talk about these.
    private Trajectory slalomPathSect1;
    private Trajectory slalomPathSect2;
    private Trajectory slalomPathSect3;
    private Trajectory slalomPathSect4;

    public Trajectories(DriveTrain robot) {
        // Setup the config for the galatic search paths
        config = new TrajectoryConfig(kVelocityMax, kAccelerationMax);
        specialConfig = new TrajectoryConfig(kSpecialVelocityMax, kSpecialAccelerationMax);
        specialConfig.setReversed(true);
        config.setKinematics(robot.getKinematics());
        // Trajectories
        pathARed = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(0)),
                new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-2.5)),
                new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(5))
            ),
            new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
            // Pass config
            config
        );

        pathBRed = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.feetToMeters(5), Units.feetToMeters(2.5)),
                new Translation2d(Units.feetToMeters(11), Units.feetToMeters(-2.0)),
                new Translation2d(Units.feetToMeters(15), Units.feetToMeters(2.5))
            ),
            new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
            // Pass config
            config
        );

        pathABlue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-4)),
                new Translation2d(Units.feetToMeters(16), Units.feetToMeters(3.5)),
                new Translation2d(Units.feetToMeters(20.0), Units.feetToMeters(1))
            ),
            new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
            // Pass config
            config
        );

        pathBBlue = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-2.5)),
                new Translation2d(Units.feetToMeters(20.5), Units.feetToMeters(3.5)),
                new Translation2d(Units.feetToMeters(24.5), Units.feetToMeters(-2.5))
            ),
            new Pose2d(Units.feetToMeters(30), Units.feetToMeters(0), new Rotation2d(0)),
            // Pass config
            config
        );

        // Barrel Racing
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kBarrelRacePath);
            barrelRacingPath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kBarrelRacePath, ex.getStackTrace());
        }

        // Bounce
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kBouncePath);
            bouncePath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kBouncePath, ex.getStackTrace());
        }

        // Slalom
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kSlalomPath);
            slalomPath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kSlalomPath, ex.getStackTrace());
        }

        // Test path in case of debug
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kTestPath);
        //     pathweaverTestPath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory: " + kTestPath, ex.getStackTrace());
        // }
        pathweaverTestPath = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.feetToMeters(-1), Units.feetToMeters(0), new Rotation2d(0)),
            // Pass config
            specialConfig
        );

        // Slalom Sections
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kSlalomPathnamePt1);
            slalomPathSect1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kSlalomPathnamePt1, ex.getStackTrace());
        }

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kSlalomPathnamePt2);
            slalomPathSect2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kSlalomPathnamePt2, ex.getStackTrace());
        }

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kSlalomPathnamePt3);
            slalomPathSect3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kSlalomPathnamePt3, ex.getStackTrace());
        }

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(kSlalomPathnamePt4);
            slalomPathSect4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + kSlalomPathnamePt4, ex.getStackTrace());
        }
    }

    public Trajectory getPathARed() {
        return pathARed;
    }

    public Trajectory getPathBRed() {
        return pathBRed;
    }

    public Trajectory getPathABlue() {
        return pathABlue;
    }

    public Trajectory getPathBBlue() {
        return pathBBlue;
    }

    public Trajectory getBarrelRacingPath() {
        return barrelRacingPath;
    }

    public Trajectory getBouncePath() {
        return bouncePath;
    }

    public Trajectory getSlalomPath() {
        return slalomPath;
    }

    public Trajectory getTestPath() {
        return pathweaverTestPath;
    }

    public Trajectory getSlalomPathSect(int sect) {
        if(sect == 1) {
            return slalomPathSect1;
        }
        if(sect == 2) {
            return slalomPathSect2;
        }
        if(sect == 3) {
            return slalomPathSect3;
        }
        if(sect == 4) {
            return slalomPathSect4;
        }
        return null;
    }
}