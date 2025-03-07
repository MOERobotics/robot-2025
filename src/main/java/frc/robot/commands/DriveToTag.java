

package frc.robot.commands;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystem.SwerveDrive;
import edu.wpi.first.math.geometry.*;


import java.util.ArrayList;

import static edu.wpi.first.units.Units.Inches;


public class DriveToTag extends Command {
    // Figures out current location and desired location
    // Drives to that new location


    static ArrayList<Pose2d> poses = new ArrayList<>() {{

        // ID 17
        Pose2d targetPose1 = new Pose2d(Units.inchesToMeters(160.3900), Units.inchesToMeters(130.1700), Rotation2d.fromDegrees(240));
        Pose2d targetPose1Transformed = new Pose2d(Units.inchesToMeters(160.3900), Units.inchesToMeters(130.1700), Rotation2d.fromDegrees(240));

        // ID 18
        Pose2d targetPose2 = new Pose2d(Units.inchesToMeters(144.0000), Units.inchesToMeters(158.5000), Rotation2d.fromDegrees(180));
        // ID 19
        Pose2d targetPose3 = new Pose2d(Units.inchesToMeters(160.3900), Units.inchesToMeters(186.8300), Rotation2d.fromDegrees(120));
        // ID 20
        Pose2d targetPose4 = new Pose2d(Units.inchesToMeters(193.1000), Units.inchesToMeters(186.8300), Rotation2d.fromDegrees(60));
        // ID 21
        Pose2d targetPose5 = new Pose2d(Units.inchesToMeters(209.4900), Units.inchesToMeters(158.5000), Rotation2d.fromDegrees(0));
        // ID 22
        Pose2d targetPose6 = new Pose2d(Units.inchesToMeters(193.1000), Units.inchesToMeters(130.1700), Rotation2d.fromDegrees(300));
        add(targetPose1Transformed);
        //add(targetPose2);
        //add(targetPose3);
        //add(targetPose4);
        //add(targetPose5);
        //add(targetPose6);
       //add(LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d());



        // ID 11
        Pose2d targetPose7 = new Pose2d(Units.inchesToMeters(497.7700), Units.inchesToMeters(130.1700), Rotation2d.fromDegrees(240));
        // ID 10
        Pose2d targetPose8 = new Pose2d(Units.inchesToMeters(481.3900), Units.inchesToMeters(158.5000), Rotation2d.fromDegrees(180));
        // ID 9
        Pose2d targetPose9 = new Pose2d(Units.inchesToMeters(497.7700), Units.inchesToMeters(186.8300), Rotation2d.fromDegrees(120));
        // ID 8
        Pose2d targetPose10 = new Pose2d(Units.inchesToMeters(530.4900), Units.inchesToMeters(186.8300), Rotation2d.fromDegrees(60));
        // ID 7
        Pose2d targetPose11 = new Pose2d(Units.inchesToMeters(546.8700), Units.inchesToMeters(158.5000), Rotation2d.fromDegrees(0));
        // ID 6
        Pose2d targetPose12 = new Pose2d(Units.inchesToMeters(530.4900), Units.inchesToMeters(130.1700), Rotation2d.fromDegrees(300));
        //add(targetPose7);
        //add(targetPose8);
        //add(targetPose9);
        //add(targetPose10);
        //add(targetPose11);
        //add(targetPose12);

    }};
    public static Pose2d getClosestTarget(Pose2d current) {
        Pose2d nearestTag = current.nearest(poses);
        return nearestTag.plus(new Transform2d(Inches.of(15),Inches.of(0),Rotation2d.k180deg));
        //return nearestTag;
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
