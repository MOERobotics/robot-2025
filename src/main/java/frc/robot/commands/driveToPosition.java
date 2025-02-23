package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystem.SwerveDrive;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystem.SwerveDriveControl;

import java.util.ArrayList;

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
                } else {
                    // TODO: Panic if we still don't have an answer after like 3sec
                    System.out.println("Heartbeat unchanged, waiting");
                }
                return;
            } else if (tag_count == 5) {
                // TODO: angle wrapping
                double x=0, y=0, zr=0;
                for (Pose2d pose : found_poses) {
                    x  += pose.getX();
                    y  += pose.getY();
                    zr += pose.getRotation().getDegrees();
                }
                x  /= found_poses.length;
                y  /= found_poses.length;
                zr /= found_poses.length;

                Pose2d average_pose = new Pose2d(x,y,Rotation2d.fromDegrees(zr));
                /// idfk
                chosenPose = average_pose;
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