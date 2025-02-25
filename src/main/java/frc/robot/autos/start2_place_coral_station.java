package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralHeadAutoCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.container.RobotContainer;
import lombok.SneakyThrows;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;

public class start2_place_coral_station {

    public  static Command buildS2EL4Command(RobotContainer robot) {
        return generateAutos(robot, "Start2 E", "E Coral Station");
    }
    @SneakyThrows
    public static Command generateAutos(RobotContainer robot, String path1, String path2 ){
        PathPlannerPath plannerPath1 = PathPlannerPath.fromPathFile(path1);
        PathPlannerPath plannerPath2 = PathPlannerPath.fromPathFile(path2);
        return Commands.sequence(
                Commands.runOnce(()->robot.getSwerveDrive().resetPose(plannerPath1.getStartingHolonomicPose().get())),
                Commands.deadline(
                        AutoBuilder.followPath(plannerPath1)
                        //new ElevatorAutoCommand(robot.getElevator(), 2, InchesPerSecond.of(9),true)
                ),
                //new ElevatorAutoCommand(robot.getElevator(), 3, InchesPerSecond.of(9),false),
                Commands.deadline(
                    new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(2)
                    //new ElevatorAutoCommand(robot.getElevator(), 3, InchesPerSecond.of(9),true)
                )
                //new ElevatorAutoCommand(robot.getElevator(),5, InchesPerSecond.of(8),false)
                //, AutoBuilder.followPath(plannerPath2)
        );
    }
}
