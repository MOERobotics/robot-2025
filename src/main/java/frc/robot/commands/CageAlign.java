package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.interfaces.SwerveDriveControl;

import java.util.List;

public class CageAlign extends Command {
    // Figures out current location and desired location
    // Drives to that new location

    private final SwerveDriveControl swerveDrive;
    private Command generateTrajectory;
    private Joystick joystick;

    public CageAlign(SwerveDriveControl swerveDrive, Joystick joystick) {
        this.swerveDrive = swerveDrive;
        generateTrajectory = null;
        this.joystick = joystick;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        Rotation2d pathDir = swerveDrive.getPose().getX()>8.775?Rotation2d.kPi:Rotation2d.kZero;
        Pose2d target = new Pose2d(8.775,swerveDrive.getPose().getY(),pathDir);

        Rotation2d endRot = (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)== DriverStation.Alliance.Blue)?Rotation2d.fromDegrees(90):Rotation2d.fromDegrees(-90);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(swerveDrive.getPose().getTranslation(),pathDir), target);
        PathPlannerPath trajectory = new PathPlannerPath(waypoints, new PathConstraints(1.5, 2.5, Units.degreesToRadians(360), Units.degreesToRadians(720)), null, new GoalEndState(0,endRot));
        trajectory.preventFlipping = true;
        generateTrajectory = AutoBuilder.followPath(trajectory);
        generateTrajectory.handleInterrupt(()->swerveDrive.drive(0,0,0)).withTimeout(2).schedule();

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