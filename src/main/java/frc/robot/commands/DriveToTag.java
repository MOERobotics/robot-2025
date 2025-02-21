

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

        /*
        // ID 17
        Pose2d targetPose1 = new Pose2d(160.3900, 130.1700, targetRotation);
        // ID 18
        Pose2d targetPose2 = new Pose2d(144.0000, 158.5000, targetRotation);
        // ID 19
        Pose2d targetPose3 = new Pose2d(160.3900, 186.8300, targetRotation);
        // ID 20
        Pose2d targetPose4 = new Pose2d(193.1000, 186.8300, targetRotation);
        // ID 21
        Pose2d targetPose5 = new Pose2d(209.4900, 158.5000, targetRotation);
        // ID 22
        Pose2d targetPose6 = new Pose2d(193.1000, 130.1700, targetRotation);
        add(targetPose1);
        add(targetPose2);
        add(targetPose3);
        add(targetPose4);
        add(targetPose5);
        add(targetPose6);
         */

         /*
        // ID 11
        Pose2d targetPose7 = new Pose2d(497.7700, 130.1700, targetRotation);
        // ID 10
        Pose2d targetPose8 = new Pose2d(481.3900, 158.5000, targetRotation);
        // ID 9
        Pose2d targetPose9 = new Pose2d(497.7700, 186.8300, targetRotation);
        // ID 8
        Pose2d targetPose10 = new Pose2d(530.4900, 186.8300, targetRotation);
        // ID 7
        Pose2d targetPose11 = new Pose2d(546.8700, 158.5000, targetRotation);
        // ID 6
        Pose2d targetPose12 = new Pose2d(530.4900, 130.1700, targetRotation);
        add(targetPose7);
        add(targetPose8);
        add(targetPose9);
        add(targetPose10);
        add(targetPose11);
        add(targetPose12);
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
