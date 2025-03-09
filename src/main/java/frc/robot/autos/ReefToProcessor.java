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
import frc.robot.subsystem.interfaces.ElevatorControl;
import lombok.SneakyThrows;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.*;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.LEVEL4;

public class ReefToProcessor {
    // START 3 AUTOS
    public static Command S3_G4_PRO(RobotContainer robot) {
        return buildReefToProcessorCommands(robot, "Start3 G", "G Processor", LEVEL4);
    }
    public static Command S3_H4_PRO(RobotContainer robot) {
        return buildReefToProcessorCommands(robot, "Start3 H", "H Processor", LEVEL4);
    }


    @SneakyThrows
    public static Command buildReefToProcessorCommands(
        RobotContainer robot,
        String path1,
        String path2,
        ElevatorControl.ElevatorHeight scoring_level) {
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
            // Follow path 1 & raise elevator to level 2
            Commands.deadline(
                AutoBuilder.followPath(plannerPath1).finallyDo(() -> robot.getSwerveDrive().drive(0,0,0)),
                new ElevatorAutoCommand(robot.getElevator(),  LEVEL2.measure, InchesPerSecond.of(6),true)
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

            new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, InchesPerSecond.of(10),false).withTimeout(1.0)//,

//            Commands.deadline(
//                AutoBuilder.followPath(plannerPath2),
//                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, InchesPerSecond.of(10),true)
//            ),
//            Commands.runOnce(()-> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
//            new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(1.0)).withTimeout(3)

        );
    }
}
