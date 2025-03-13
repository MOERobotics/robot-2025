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
import frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight;
import lombok.SneakyThrows;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.*;

public class ReefToSourceToReef {

    // START 1 AUTOS
    public static Command S1_C4_CS_D(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start1 C", "C Coral Station","Coral Station D", LEVEL4);
    }
    public static Command S1_B4_CS_A(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start1 B", "B Coral Station 2","Coral Station 2 A", LEVEL4);
    }
    public static Command S1_A4_CS_B(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start1 A", "A Coral Station 2","Coral Station 2 B", LEVEL4);
    }
    // START 2 AUTOS
    public static Command S2_E4_CS_F(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start2 E", "E Coral Station","Coral Station F", LEVEL4);
    }

    public static Command S2_E4_CS_C(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start2 E", "E Coral Station","Coral Station C", LEVEL4);
    }

    public static Command S2_F4_CS_C(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start2 F", "F Coral Station","Coral Station C", LEVEL4);
    }
    public static Command S2_C4_CS_D(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start2 C", "C Coral Station","Coral Station D", LEVEL4);
    }



    public static Command S2_D4_CS_C(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start2 D", "D Coral Station","Coral Station C", LEVEL4);
    }
    // START 3 AUTOS
    /*
    public static Command S3_G4_PRO(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start3 G", "G Processor", LEVEL4);
    }
    public static Command S3_H4_PRO(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start3 H", "H Processor", LEVEL4);
    }
     */
    // START 4 AUTOS
    public static Command S4_I4_CS_J(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start4 I", "I Coral Station","Coral Station J", LEVEL1);
    }
    public static Command S4_J4_CS_I(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start4 J", "J Coral Station","Coral Station I", LEVEL4);
    }
    public static Command S4_K4_CS_L(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start4 K", "K Coral Station","Coral Station L", LEVEL4);
    }
    public static Command S4_L4_CS_K(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start4 L", "L Coral Station","Coral Station K", LEVEL4);
    }
    // START 5 AUTOS
    public static Command S5_L4_CS_K(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start5 L", "L Coral Station","Coral Station K", LEVEL4);
    }
    public static Command S5_B4_CS_A(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start5 B", "B Coral Station 1","Coral Station 1 A", LEVEL4);
    }
    public static Command S5_A4_CS_B(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start5 A", "A Coral Station 1","Coral Station 1 B", LEVEL4);
    }




    @SneakyThrows
    public static Command buildReefToSourceToReefCommand(
            RobotContainer robot,
            String path1,
            String path2,
            String path3,
            ElevatorHeight scoring_level
    ) {
        PathPlannerPath plannerPath1 = PathPlannerPath.fromPathFile(path1);
        PathPlannerPath plannerPath2 = PathPlannerPath.fromPathFile(path2);
        PathPlannerPath plannerPath3 = PathPlannerPath.fromPathFile(path3);

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
                // Follow path 1 & raise elevator to level 2
                Commands.deadline(
                        AutoBuilder.followPath(plannerPath1),
                        new ElevatorAutoCommand(robot.getElevator(),  LEVEL2.measure, FeetPerSecond.of(0.25),true)
                ),
                // Raise coral to desired level & stop
                Commands.deadline(
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),false),
                        Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())
                ),
                // Dispense coral & hold at desired level TODO: Update from scoring for a time limit to beam break system
                Commands.deadline(
                        new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(0.5),
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),true)
                ),
                 //  start to move  to collect position
//                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1),false).withTimeout(1),
                // move elevator to collect position while moving to source
                Commands.deadline(
                    AutoBuilder.followPath(plannerPath2),
                    new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1),false)
                ),
                Commands.runOnce(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
                // run coral head to collect coral
                new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(1.0)),

                // move back to reef and move elevator to L2
                Commands.deadline(
                        AutoBuilder.followPath(plannerPath3).finallyDo(() -> robot.getSwerveDrive().drive(0,0,0)),
                        new ElevatorAutoCommand(robot.getElevator(), LEVEL2.measure, FeetPerSecond.of(0.25),true)
                ),
                Commands.runOnce(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
                // hold in L4
                Commands.deadline(
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),false),
                        Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())

                ),
                // score in L4
                Commands.deadline(
                        new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(0.5),
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),true)
                ),
                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1),false)

        );




    }


}
