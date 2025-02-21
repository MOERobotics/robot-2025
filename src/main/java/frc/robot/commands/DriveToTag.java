

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SwerveDrive;
import edu.wpi.first.math.geometry.*;


import java.util.ArrayList;


public class DriveToTag extends Command {
    // Figures out current location and desired location
    // Drives to that new location


    static ArrayList<Pose2d> poses = new ArrayList<>() {{
        Rotation2d targetRotation = new Rotation2d(0);
        Pose2d targetPose1 = new Pose2d(1, 1, targetRotation);
        Pose2d targetPose6 = new Pose2d(1, 4, targetRotation);
        Pose2d targetPose2 = new Pose2d(1, 7, targetRotation);
        Pose2d targetPose3 = new Pose2d(1, 11, targetRotation);
        Pose2d targetPose4 = new Pose2d(2, 1, targetRotation);
        Pose2d targetPose5 = new Pose2d(6, 1, targetRotation);
        add(targetPose1);
        /*
        add(targetPose2);
        add(targetPose3);
        add(targetPose4);
        add(targetPose5);
        add(targetPose6);
        */
    }};
    public static Pose2d getClosestTarget(Pose2d current) {
        return current.nearest(poses);
    }



    @Override
    public void initialize() {
        // targetPose = targetPose.transformBy(currentPose.minus(targetPose));


    }


    public void execute() {


    }


    @Override
    public void end(boolean interrupted) {


    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
