package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralHeadAutoCommand;
import frc.robot.commands.CoralHeadTeleopCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.commands.ElevatorTeleopCommand;
import frc.robot.container.RobotContainer;
import lombok.SneakyThrows;

import java.util.Set;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;

public class start4_place_coral_station {

    public  static Command buildS4IL4Command(RobotContainer robot) {
        return generateAutos(robot, "Start2 EF", "EF Coral Station");
    }
    @SneakyThrows
    public static Command generateAutos(RobotContainer robot, String path1, String path2 ){
        PathPlannerPath plannerPath1 = PathPlannerPath.fromPathFile(path1);
        PathPlannerPath plannerPath2 = PathPlannerPath.fromPathFile(path2);
        return Commands.sequence(
                Commands.run(()->robot.getSwerveDrive().resetPose(plannerPath1.getStartingDifferentialPose())),
            /*    Commands.parallel(
                        AutoBuilder.followPath(plannerPath1),
                        new ElevatorAutoCommand(robot.getElevator(), 4, InchesPerSecond.of(9),true)
                ),
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(1.0)).withTimeout(2),
                new ElevatorAutoCommand(robot.getElevator(),5,InchesPerSecond.of(8),true),
                */
                AutoBuilder.followPath(plannerPath2)
        );
    }
}
