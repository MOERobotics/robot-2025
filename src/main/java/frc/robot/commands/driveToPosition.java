package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystem.SwerveDrive;
import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;

public class driveToPosition extends Command {
    // Figures out current location and desired location
    // Drives to that new location

    private final SwerveDrive swerveDrive;
    private Pose2d targetPose;
    private Pose2d currentPose;
    private Command generateTrajectory;

    public driveToPosition(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        // targetPose = targetPose.transformBy(currentPose.minus(targetPose));
        currentPose = swerveDrive.getPose();
        targetPose = DriveToTag.getClosestTarget(currentPose);
        currentPose = LimelightHelpers.getBotPose2d("");
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        internalPoints.add(new Translation2d(1,1));
        generateTrajectory = swerveDrive.generateTrajectory(currentPose, targetPose, internalPoints, 0, 0);
        generateTrajectory.initialize();
    }

    public void execute() {
        generateTrajectory.execute();
    }

    @Override
    public void end(boolean interrupted) {
        generateTrajectory.end(false);
    }

    @Override
    public boolean isFinished() {
       return generateTrajectory.isFinished();
    }
}