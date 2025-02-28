package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystem.SwerveDrive;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystem.SwerveDriveControl;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class driveToPosition extends Command {
    // Figures out current location and desired location
    // Drives to that new location

    private final SwerveDriveControl swerveDrive;
    private Pose2d targetPose;
    private Pose2d currentPose;
    private Command generateTrajectory;

    private static final double[] fakeLimelightData = new double[11];


    int tag_count = 0;
    Pose2d[] found_poses = new Pose2d[5];
    Pose2d chosenPose = null;
    long last_reading = 0;
    NetworkTable limelightAPI = null;

    public driveToPosition(SwerveDriveControl swerveDrive, Pose2d targetPose) {
        this.swerveDrive = swerveDrive;
        this.targetPose = targetPose;
        Arrays.fill(found_poses, Pose2d.kZero);
    }

    @Override
    public void initialize() {
        tag_count = 0;
        chosenPose = null;
        last_reading = 0;
        generateTrajectory = null;
        limelightAPI = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void execute() {
        Logger.recordOutput("driveToPosition/chosenPose", chosenPose);
        Logger.recordOutput("driveToPosition/found_poses", found_poses);
        Logger.recordOutput("driveToPosition/last_reading", last_reading);
        Logger.recordOutput("driveToPosition/tag_count", tag_count);

        if (chosenPose == null) {
            long heartbeat = (long)limelightAPI.getEntry("hb").getDouble(0);
            if (heartbeat == 0) {
                chosenPose = swerveDrive.getPose();
            } else if (tag_count < 5) {
                if (heartbeat != last_reading) {
                    double[] limelightReading = limelightAPI.getEntry("botpose").getDoubleArray(fakeLimelightData);
                    Pose2d limelightPose = new Pose2d(
                        Meters.of(limelightReading[0]),
                        Meters.of(limelightReading[1]),
                        new Rotation2d(Degrees.of(limelightReading[5]))
                    );
                    found_poses[tag_count] = limelightPose;
                    tag_count += 1;
                    last_reading = heartbeat;
                } else {
                    // TODO: Panic if we still don't have an answer after like 3sec
                    System.out.println("Heartbeat unchanged, waiting");
                }
                return;
            } else if (tag_count >= 5) {
                // TODO: angle wrapping
                double bestDistance = Double.MAX_VALUE;
                for (int i = 0; i < 3; i++) {
                    for (int j = i+1; j < 4; j++) {
                        for (int k = j + 1; k < 5; k++) {
                            Pose2d
                                pose1 = found_poses[i],
                                pose2 = found_poses[j],
                                pose3 = found_poses[k];

                            double
                                x = (pose1.getX() + pose2.getX() + pose3.getX()) / 3.0,
                                y=0,//same
                                zr=0; //same maybe

                            Pose2d averagePose = new Pose2d(x, y, new Rotation2d(zr));

                            double distance = (
                                    (pose1.getX() - averagePose.getX()) +
                                    (pose1.getY() - averagePose.getY()) +
                                    // ((double)(pose1.getRotation()
                                    // Math.toDegrees(pose1.getRotation());

                                    (pose2.getX() - averagePose.getX()) +
                                    (pose2.getY() - averagePose.getY()) +
                                    // (pose2.getX() - averagePose.getX()) +

                                    (pose3.getX() - averagePose.getX()) +
                                    (pose3.getY() - averagePose.getY()) +
                                    (pose3.getX() - averagePose.getX())
                            );
                            if (distance < bestDistance) {
                                bestDistance = distance;
                                chosenPose = averagePose;
                            }
                        }
                    }
                }
            }
        }

        if (generateTrajectory == null) {
            generateTrajectory = swerveDrive.generateTrajectory(
                chosenPose,
                targetPose,
                new ArrayList<Translation2d>(),
                0,
                0
            );
            generateTrajectory.initialize();
        }
        generateTrajectory.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (generateTrajectory != null) generateTrajectory.end(false);
    }

    @Override
    public boolean isFinished() {
       return generateTrajectory != null && generateTrajectory.isFinished();
    }
}