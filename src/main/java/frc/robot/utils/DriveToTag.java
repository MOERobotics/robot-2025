

package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.Inches;


public class DriveToTag {

    //Pose of all Reef April Tags
    static ArrayList<Pose2d> poses = new ArrayList<>() {{

        //Blue Reef
        // ID 17
        Pose2d targetPose1 = new Pose2d(Units.inchesToMeters(160.3900), Units.inchesToMeters(130.1700), Rotation2d.fromDegrees(240));
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
        add(targetPose1);
        add(targetPose2);
        add(targetPose3);
        add(targetPose4);
        add(targetPose5);
        add(targetPose6);


        //Red Reef
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
        add(targetPose7);
        add(targetPose8);
        add(targetPose9);
        add(targetPose10);
        add(targetPose11);
        add(targetPose12);
    }};

    public static Pose2d getClosestTarget(Pose2d currentPose, Transform2d transformation) {
        Pose2d nearestTag = currentPose.nearest(poses);
        return nearestTag.plus(transformation);
    }

    public static Pose2d getClosestTarget(Pose2d currentPose) {
        return getClosestTarget(currentPose, new Transform2d(Inches.of(15), Inches.of(0), Rotation2d.k180deg));
    }
}