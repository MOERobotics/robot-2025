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

public class ReefToSourceToReef {


    public static Command S2_E2_CS_C2(RobotContainer robot) {
        return buildReefToSourceToReefCommand(robot, "Start2 E", "E Coral Station","Coral Station To C", LEVEL2);
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

        return Commands.sequence(
                Commands.runOnce(()->robot.getSwerveDrive().resetPose(plannerPath1.getStartingHolonomicPose().get())),
                Commands.deadline(
                        AutoBuilder.followPath(plannerPath1).finallyDo(() -> robot.getSwerveDrive().drive(0,0,0)),
                        new ElevatorAutoCommand(robot.getElevator(), LEVEL4.measure, InchesPerSecond.of(5),true)
                ),
                Commands.deadline(
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),false),
                        Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())

                ),
                Commands.deadline(
                        new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(2),
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),true)
                ),
                // new ElevatorAutoCommand(robot.getElevator(),STOW.measure, InchesPerSecond.of(8),false),
                AutoBuilder.followPath(plannerPath2),
                new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(1.0)).withTimeout(2),
                Commands.deadline(
                        AutoBuilder.followPath(plannerPath3).finallyDo(() -> robot.getSwerveDrive().drive(0,0,0)),
                        new ElevatorAutoCommand(robot.getElevator(), LEVEL2.measure, InchesPerSecond.of(9),true)
                ),
                Commands.deadline(
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),false),
                        Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive())

                ),
                Commands.deadline(
                        new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(2),
                        new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, InchesPerSecond.of(9),true)
                )



        );
    }


}
