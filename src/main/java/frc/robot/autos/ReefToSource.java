package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralHeadAutoCommand;
import frc.robot.commands.ElevatorAutoCommand;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight;
import lombok.SneakyThrows;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystem.interfaces.ElevatorControl.ElevatorHeight.*;

public class ReefToSource {
// START 1 AUTOS
    public static Autos.CommandAndPose S1_C4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 C", "C Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S1_B4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 B", "B Coral Station 2", LEVEL4);
    }
    public static Autos.CommandAndPose S1_A4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start1 A", "A Coral Station 2", LEVEL4);
    }
    // START 2 AUTOS
    public static Autos.CommandAndPose S2_E4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 E", "E Coral Station", LEVEL4);
    }


    public static Autos.CommandAndPose S2_F4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 F", "F Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S2_C4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 C", "C Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S2_D4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start2 D", "D Coral Station", LEVEL4);
    }
    // START 4 AUTOS
    public static Autos.CommandAndPose S4_I4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 I", "I Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S4_J4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 J", "J Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S4_K4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 K", "K Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S4_L4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start4 L", "L Coral Station", LEVEL4);
    }
// START 5 AUTOS
    public static Autos.CommandAndPose S5_L4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 L", "L Coral Station", LEVEL4);
    }
    public static Autos.CommandAndPose S5_B4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 B", "B Coral Station 1", LEVEL4);
    }
    public static Autos.CommandAndPose S5_A4_CS(RobotContainer robot) {
        return buildReefToSourceCommand(robot, "Start5 A", "A Coral Station 1", LEVEL4);
    }



    @SneakyThrows
    public static Autos.CommandAndPose buildReefToSourceCommand(
        RobotContainer robot,
        String path1,
        String path2,
        ElevatorHeight scoring_level
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
        return new Autos.CommandAndPose(Commands.sequence(
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
                new CoralHeadAutoCommand(robot.getCoralHead(), true, RPM.of(0.95)).withTimeout(0.3),
                new ElevatorAutoCommand(robot.getElevator(), scoring_level.measure, FeetPerSecond.of(1),true)
            ),
//            new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, InchesPerSecond.of(10),false).withTimeout(1.0),
            // move elevator to collect position while moving to source
            Commands.deadline(
                AutoBuilder.followPath(plannerPath2),
                new ElevatorAutoCommand(robot.getElevator(), COLLECT.measure, FeetPerSecond.of(1),true)
            ),
            Commands.runOnce(()-> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()),
            new CoralHeadAutoCommand(robot.getCoralHead(), false, RPM.of(0.4))

        ),startingPose);
    }




}
