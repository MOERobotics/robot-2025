/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.autos.Autos;
import frc.robot.autos.start4_place_coral_station;
import frc.robot.commands.*;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.simulations.*;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.commands.junk.SwerveModuleTestingCommand;


public class Robot extends LoggedRobot {


    CommandJoystick driverJoystick = new CommandJoystick(0);
    CommandJoystick functionJoystick = new CommandJoystick(1);
    CommandScheduler scheduler;


    RobotContainer robot = new SubMOErine();
    Command autoCommand = Commands.none();

    Field2d field = new Field2d();

    PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    @Override
    public void robotInit() {
        SmartDashboard.putData(field);
        PathPlannerLogging.setLogActivePathCallback(path -> {
            field.getObject("traj").setPoses(path);
        });
        PathPlannerLogging.setLogCurrentPoseCallback(path -> {
            field.getRobotObject().setPose(path);
        });
        PathPlannerLogging.setLogTargetPoseCallback(path -> {
            field.getObject("target").setPose(path);
        });

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);

        MOELogger.setupLogging(this);

        scheduler = CommandScheduler.getInstance();
        Autos.setupAutos(robot);
    }


    @Override
    public void robotPeriodic() {
        scheduler.run();
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        scheduler.cancelAll();
        this.autoCommand = start4_place_coral_station.buildS4IL4Command(robot);
        var swerveDrive = robot.getSwerveDrive();
        Command stopCommand = swerveDrive.run(()-> { swerveDrive.drive(0,0,0);});
        stopCommand.setName("SwerveStop");
        swerveDrive.setDefaultCommand(stopCommand);
        autoCommand = Autos.getSelectedAuto();
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        scheduler.run();
    }

    @Override
    public void teleopInit() {
        scheduler.cancelAll();

        new ElevatorTeleopCommand(
            robot.getElevator(),
            functionJoystick.getHID()
        ).schedule();
        new AlgaeCollectorTeleopCommand(
            robot.getAlgaeCollector(),
            functionJoystick.getHID()
        ).schedule();
        new CoralHeadTeleopCommand(
            robot.getCoralHead(),
            functionJoystick.getHID(),
            robot.getElevator()
        ).schedule();
        new ClimberTeleopCommand(
            robot.getClimberRear(),
            robot.getClimberMid(),
            driverJoystick.getHID()
        ).schedule();
        robot.getSwerveDrive().setDefaultCommand( new SwerveControllerCommand(robot.getSwerveDrive(), driverJoystick.getHID()));
    }

    @Override
    public void teleopPeriodic() {
        scheduler.run();
        if(driverJoystick.getHID().getRawButtonPressed(1)){
            robot.getSwerveDrive().resetPose(new Pose2d());
        }
        functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,0);
        if (robot.getCoralHead().inFrontReef()){
            SmartDashboard.putBoolean("DROP", true);
            pdh.setSwitchableChannel(true);
            functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,1);
        }
        else {
            SmartDashboard.putBoolean("DROP", false);
            pdh.setSwitchableChannel(false);
            functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void testInit() {
        scheduler.cancelAll();
        robot.getSwerveDrive().setDefaultCommand(new SwerveModuleTestingCommand(robot.getSwerveDrive(), driverJoystick.getHID()));

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {
        SimulationHelpers.setUpSim(robot);
    }

    @Override
    public void simulationPeriodic() {
        SimulationHelpers.updateSimState();
    }
}
