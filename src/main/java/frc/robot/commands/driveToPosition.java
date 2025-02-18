package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.LimeLights;
import frc.robot.subsystem.LimelightHelpers;
import frc.robot.subsystem.SwerveDrive;
import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;

public class driveToPosition extends Command {
    // Figures out current location and desired location
    // Drives to that new location

    private final SwerveDrive swerveDrive;
    private Pose2d targetPose;
    private Pose2d currentPose;

    public driveToPosition(SwerveDrive swerveDrive, Pose2d targetPose, Pose2d currentPose) {
        this.swerveDrive = swerveDrive;
        this.currentPose = currentPose;
    }

    @Override
    public void initialize() {
       // targetPose = targetPose.transformBy(currentPose.minus(targetPose));
        currentPose = LimelightHelpers.getBotPose2d("");
    }

    public void execute() {
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

        swerveDrive.generateTrajectory(currentPose, targetPose, internalPoints, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

