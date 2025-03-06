package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralHeadAutoCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ElevatorControl;
import frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight;
import lombok.SneakyThrows;
import lombok.val;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.*;

public class ReefToSource {
// START 1 AUTOS
    public static Command S1_C4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 C", "C Coral Station", LEVEL4);
    }
    public static Command S1_B4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 B", "B Coral Station 2", LEVEL4);
    }
    public static Command S1_A4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 A", "A Coral Station 2", LEVEL4);
    }
    // START 2 AUTOS
    public static Command S2_E4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 E", "E Coral Station", LEVEL4);
    }


    public static Command S2_F4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 F", "F Coral Station", LEVEL4);
    }
    public static Command S2_C4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 C", "C Coral Station", LEVEL4);
    }
    public static Command S2_D4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 D", "D Coral Station", LEVEL4);
    }
    // START 3 AUTOS
    public static Command S3_G4_PRO(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start3 G", "G Processor", LEVEL4);
    }
    public static Command S3_H4_PRO(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start3 H", "H Processor", LEVEL4);
    }
    // START 4 AUTOS
    public static Command S4_I4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 I", "I Coral Station", LEVEL4);
    }
    public static Command S4_J4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 J", "J Coral Station", LEVEL4);
    }
    public static Command S4_K4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 K", "K Coral Station", LEVEL4);
    }
    public static Command S4_L4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 L", "L Coral Station", LEVEL4);
    }
// START 5 AUTOS
    public static Command S5_L4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 L", "L Coral Station", LEVEL4);
    }
    public static Command S5_B4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 B", "B Coral Station 1", LEVEL4);
    }
    public static Command S5_A4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 A", "A Coral Station 1", LEVEL4);
    }



    @SneakyThrows
    public static Command buildReefToSourceCommand(
        RobotContainer robot,
        String path1,
        String path2,
        ElevatorHeight scoring_level
    ) {
        PathPlannerPath plannerPath1 = PathPlannerPath.fromPathFile(path1);
        PathPlannerPath plannerPath2 = PathPlannerPath.fromPathFile(path2);

        return Commands.sequence(
            // reset pose
            Commands.runOnce(()->robot.getSwerveDrive().resetPose(plannerPath1.getStartingHolonomicPose().get())),
            // Follow path 1 & raise elevator to level 2
            Commands.deadline(
                AutoBuilder.followPath(plannerPath1).finallyDo(() -> robot.getSwerveDrive().drive(0,0,0)),
                new ElevatorAutoCommand(robot.getElevator(),  LEVEL4.measure, InchesPerSecond.of(6),true)
            ),
            // Raise coral to desired level & stop
            Commands.deadline(
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),false),
                Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())
            ),
            // Dispense coral & hold at desired level TODO: Update from scoring for a time limit to bean break system
            Commands.deadline(
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(1),
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),true)
            ),

            new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, InchesPerSecond.of(10),false).withTimeout(1.5),

            Commands.deadline(
                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, InchesPerSecond.of(10),true),
                AutoBuilder.followPath(plannerPath2)
            ),
            Commands.runOnce(()-> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
            new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(1.0)).withTimeout(4)

        );
    }




}
