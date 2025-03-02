package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystem.interfaces.SwerveDriveControl;

import java.util.ArrayList;

public class driveToPosition2 extends Command {
    // Figures out current location and desired location
    // Drives to that new location

    private final SwerveDriveControl swerveDrive;
    private Pose2d targetPose;
    private Pose2d currentPose;
    private Command generateTrajectory;

    private static final double[] fakeLimelightData = new double[11];


    int tag_count = 0;
    Pose2d chosenPose = null;

    public driveToPosition2(SwerveDriveControl swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = DriveToTag.getClosestTarget(new Pose2d());
        tag_count = 0;
        chosenPose = null;
        generateTrajectory = null;
        generateTrajectory = swerveDrive.generateTrajectory(
                swerveDrive.getPose(),
                targetPose,
                new ArrayList<Translation2d>(),
                0,
                0
        );
        generateTrajectory.initialize();
    }

    public void execute() {

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