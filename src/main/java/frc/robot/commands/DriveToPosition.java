package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.utils.DriveToTag;

import java.util.List;

public class DriveToPosition extends Command {
    // Figures out current location and desired location
    // Drives to that new location

    private final SwerveDriveControl swerveDrive;
    private Command generateTrajectory;
    private boolean goRight;

    public DriveToPosition(SwerveDriveControl swerveDrive, boolean goRight) {
        this.swerveDrive = swerveDrive;
        generateTrajectory = null;
        this.goRight = goRight;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Pose2d target = DriveToTag.getClosestTarget(swerveDrive.getPose());

        if (goRight){
            target = DriveToTag.moveToRight(swerveDrive.getPose());
        }
        else{
            target = DriveToTag.moveToLeft(swerveDrive.getPose());
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(swerveDrive.getPose(), target);
        PathPlannerPath trajectory = new PathPlannerPath(waypoints, new PathConstraints(1.5, 1.25, Units.degreesToRadians(180), Units.degreesToRadians(360)), null, new GoalEndState(0.0, DriveToTag.getClosestTarget(swerveDrive.getPose()).getRotation()));
        trajectory.preventFlipping = true;
        generateTrajectory = AutoBuilder.followPath(trajectory);
        generateTrajectory.handleInterrupt(()->swerveDrive.drive(0,0,0)).withTimeout(5).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (generateTrajectory != null) {
            generateTrajectory.end(true);
        }
        swerveDrive.drive(0,0,0);
    }

    @Override
    public boolean isFinished() {
        return generateTrajectory != null && generateTrajectory.isFinished();
    }
}