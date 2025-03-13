package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralHeadAutoCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.container.RobotContainer;
import lombok.SneakyThrows;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.*;

public class ReefToSourceL1 {
    // START 1 AUTOS
    public static Command S1_C4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 C", "C Coral Station");
    }
    public static Command S1_B4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 B", "B Coral Station 2");
    }
    public static Command S1_A4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 A", "A Coral Station 2");
    }
    // START 2 AUTOS
    public static Command S2_E4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 E", "E Coral Station");
    }


    public static Command S2_F4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 F", "F Coral Station");
    }
    public static Command S2_C4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 C", "C Coral Station");
    }
    public static Command S2_D4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 D", "D Coral Station");
    }
    // START 3 AUTOS
    public static Command S3_G2_PRO(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start3 G", "G Processor");
    }
    public static Command S3_H2_PRO(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start3 H", "H Processor");
    }
    // START 4 AUTOS
    public static Command S4_I1_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 I", "I Coral Station");
    }
    public static Command S4_J4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 J", "J Coral Station");
    }
    public static Command S4_K4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 K", "K Coral Station");
    }
    public static Command S4_L4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 L", "L Coral Station");
    }
    // START 5 AUTOS
    public static Command S5_L4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 L", "L Coral Station");
    }
    public static Command S5_B4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 B", "B Coral Station 1");
    }
    public static Command S5_A4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 A", "A Coral Station 1");
    }



    @SneakyThrows
    public static Command buildReefToSourceCommand(
            RobotContainer robot,
            String path1,
            String path2
            
    ) {
        PathPlannerPath plannerPath1 = PathPlannerPath.fromPathFile(path1);
        PathPlannerPath plannerPath2 = PathPlannerPath.fromPathFile(path2);

        //Flip Pose if needed
        Pose2d startingPoseBlue = plannerPath1.getStartingHolonomicPose().get();
        final Pose2d startingPose;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            startingPose = FlippingUtil.flipFieldPose(startingPoseBlue);
        } else {
            startingPose = startingPoseBlue;
        }
        return Commands.sequence(
                // reset pose
                Commands.runOnce(()->robot.getSwerveDrive().resetPose(startingPose)),
                // Follow path 1 & raise elevator to level 1
                Commands.deadline(
                    AutoBuilder.followPath(plannerPath1),
                    new ElevatorAutoCommand(robot.getElevator(), LEVEL1.measure, FeetPerSecond.of(0.25), true)
                ),

                // Raise coral to desired level & stop
                Commands.deadline(
                    new ElevatorAutoCommand(robot.getElevator(), LEVEL1.measure, FeetPerSecond.of(1),false),
                    Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())
                ),
                // Dispense coral & hold at desired level TODO: Update from scoring for a time limit to bean break system
                Commands.deadline(
                    new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(1),
                    new ElevatorAutoCommand(robot.getElevator(), LEVEL1.measure, FeetPerSecond.of(1),true)
                ),

                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1),false)//,

//                Commands.deadline(
//                    AutoBuilder.followPath(plannerPath2),
//                    new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, InchesPerSecond.of(10),true)
//                ),
//                Commands.runOnce(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())


        );
    }




}
