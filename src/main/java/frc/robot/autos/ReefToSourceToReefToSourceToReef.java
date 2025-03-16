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
//
public class ReefToSourceToReefToSourceToReef {

    // START 1 AUTOS
    public static Command S1_C4_CS_D4_CS_B4(RobotContainer robot) {
        return buildReefToSourceToReefToSourceToReefCommand(robot, "Start1 C", "C Coral Station","Coral Station D", "D Coral Station", "Coral Station 2 B", LEVEL4);
    }
    public static Command S1_B4_CS_A4_CS_C4(RobotContainer robot) {
        return buildReefToSourceToReefToSourceToReefCommand(robot, "Start1 B", "B Coral Station 2","Coral Station 2 A", "A Coral Station 2", "Coral Station C", LEVEL4);
    }
    // START 2 AUTOS
    public static Command S2_E4_CS_C4_CS_D4(RobotContainer robot) {
        return buildReefToSourceToReefToSourceToReefCommand(robot, "Start2 E", "E Coral Station","Coral Station C", "C Coral Station", "Coral Station D", LEVEL4);
    }
    public static Command S2_F4_CS_C4_CS_D4(RobotContainer robot) {
        return buildReefToSourceToReefToSourceToReefCommand(robot, "Start2 F", "F Coral Station","Coral Station C", "C Coral Station", "Coral Station D", LEVEL4);
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
    public static Command S4_I4_CS_K4_CS_L4(RobotContainer robot) {
        return buildReefToSourceToReefToSourceToReefCommand(robot, "Start4 I", "I Coral Station","Coral Station K", "K Coral Station", "Coral Station L", LEVEL4);
    }
    // START 5 AUTOS
    public static Command S5_K4_CS_L4_CS_A4(RobotContainer robot) {
        return buildReefToSourceToReefToSourceToReefCommand(robot, "Start5 K", "K Coral Station","Coral Station L", "L Coral Station", "Coral Station 1 A", LEVEL4);
    }




    @SneakyThrows
    public static Command buildReefToSourceToReefToSourceToReefCommand(
        RobotContainer robot,
        String path1,
        String path2,
        String path3,
        String path4,
        String path5,
        ElevatorHeight scoring_level
    ) {
        PathPlannerPath plannerPath1 = PathPlannerPath.fromPathFile(path1);
        PathPlannerPath plannerPath2 = PathPlannerPath.fromPathFile(path2);
        PathPlannerPath plannerPath3 = PathPlannerPath.fromPathFile(path3);
        PathPlannerPath plannerPath4 = PathPlannerPath.fromPathFile(path4);
        PathPlannerPath plannerPath5 = PathPlannerPath.fromPathFile(path5);

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
                new ElevatorAutoCommand(robot.getElevator(),  LEVEL3.measure, FeetPerSecond.of(0.7),true).withTimeout(1.5).
                    andThen(new ElevatorAutoCommand(robot.getElevator(),  scoring_level.measure, FeetPerSecond.of(0.7),true))
            ),
            // Raise coral to desired level & stop
            Commands.deadline(
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),false),
                Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())
            ),
            // Dispense coral & hold at desired level TODO: Update from scoring for a time limit to beam break system
            Commands.deadline(
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(0.3),
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
            new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(0.4)),

            // move back to reef and move elevator to L2
            Commands.deadline(
                AutoBuilder.followPath(plannerPath3),
                new ElevatorAutoCommand(robot.getElevator(), LEVEL3.measure, FeetPerSecond.of(0.7),true).withTimeout(1.5).
                    andThen(new ElevatorAutoCommand(robot.getElevator(),  scoring_level.measure, FeetPerSecond.of(0.7),true))
            ),
            Commands.runOnce(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
            // hold in L4
            Commands.deadline(
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),false),
                Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())
            ),
            // score in L4
            Commands.deadline(
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(0.3),
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),true)
            ),
            // move elevator to collect position while moving to source
            Commands.deadline(
                AutoBuilder.followPath(plannerPath4),
                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1),false)
            ),
            Commands.runOnce(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
            new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(0.4)),
            // move back to reef and move elevator to L2
            Commands.deadline(
                AutoBuilder.followPath(plannerPath5),
                new ElevatorAutoCommand(robot.getElevator(), LEVEL3.measure, FeetPerSecond.of(0.7),true).withTimeout(1.5).
                    andThen(new ElevatorAutoCommand(robot.getElevator(),  scoring_level.measure, FeetPerSecond.of(0.7),true))
            ),
            Commands.runOnce(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
            // hold in L4
            Commands.deadline(
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),false),
                Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())

            ),
            // score in L4
            Commands.deadline(
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(0.3),
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),true)
            ),
            new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1), false)
        );
    }


}
