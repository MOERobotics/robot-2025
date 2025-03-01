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
    public static Command S1_C2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 C", "C Coral Station", LEVEL2);
    }
    public static Command S1_B2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 B", "B Coral Station 2", LEVEL2);
    }
    public static Command S1_A2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 A", "A Coral Station 2", LEVEL2);
    }
    // START 2 AUTOS
    public static Command S2_E2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 E", "E Coral Station", LEVEL2);
    }
    public static Command S2_F2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 F", "F Coral Station", LEVEL2);
    }
    public static Command S2_C2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 C", "C Coral Station", LEVEL2);
    }
    public static Command S2_D2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 D", "D Coral Station", LEVEL2);
    }
    // START 3 AUTOS
    public static Command S3_G2_PRO(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start3 G", "G Processor", LEVEL2);
    }
    public static Command S3_H2_PRO(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start3 H", "H Processor", LEVEL2);
    }
    // START 4 AUTOS
    public static Command S4_I2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 I", "I Coral Station", LEVEL2);
    }
    public static Command S4_J2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 J", "J Coral Station", LEVEL2);
    }
    public static Command S4_K2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 K", "K Coral Station", LEVEL2);
    }
    public static Command S4_L2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 L", "L Coral Station", LEVEL2);
    }
// START 5 AUTOS
    public static Command S5_L2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 L", "L Coral Station", LEVEL2);
    }
    public static Command S5_B2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 B", "B Coral Station 1", LEVEL2);
    }
    public static Command S5_A2_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 A", "A Coral Station 1", LEVEL2);
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
            Commands.runOnce(()->robot.getSwerveDrive().resetPose(plannerPath1.getStartingHolonomicPose().get())),
            Commands.deadline(
                AutoBuilder.followPath(plannerPath1).finallyDo(() -> robot.getSwerveDrive().drive(0,0,0)),
                new ElevatorAutoCommand(robot.getElevator(), LEVEL2.measure, InchesPerSecond.of(9),true)
            ),
            Commands.deadline(
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),false),
                Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())

            ),
            Commands.deadline(
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(2),
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),true)
            ),
            new ElevatorAutoCommand(robot.getElevator(),STOW.measure, InchesPerSecond.of(8),false),
            AutoBuilder.followPath(plannerPath2)
        );
    }
}
