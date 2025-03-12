/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.autos.Autos;
import frc.robot.container.*;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.simulations.*;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.commands.junk.SwerveModuleTestingCommand;

import java.io.IOException;


public class Robot extends LoggedRobot {

    CommandJoystick driverJoystick = new CommandJoystick(0);
    CommandJoystick functionJoystick = new CommandJoystick(1);
    CommandScheduler scheduler;

    RobotContainer robot = new SubMOErine();
    Command autoCommand = Commands.none();



    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);

        MOELogger.setupLogging(this);

        scheduler = CommandScheduler.getInstance();
    }


    @Override
    public void driverStationConnected() {
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
        autoCommand = Autos.getSelectedAuto();
        autoCommand.schedule();
        robot.getSwerveDrive().setDefaultCommand(Commands.run(() -> robot.getSwerveDrive().drive(0,0,0), robot.getSwerveDrive()));
    }

    @Override
    public void autonomousPeriodic() {
        scheduler.run();
    }

    @Override
    public void teleopInit() {
        scheduler.cancelAll();
        new ElevatorTeleopCommand(
            robot,
            functionJoystick.getHID()
        ).schedule();
        new AlgaeCollectorTeleopCommand(
            robot,
            functionJoystick.getHID()
        ).schedule();
        new CoralHeadTeleopCommand(
            robot,
            functionJoystick.getHID()
        ).schedule();
        new ClimberTeleopCommand(
            robot,
            driverJoystick.getHID()
        ).schedule();
        robot.getSwerveDrive().setDefaultCommand(
            new SwerveControllerCommand(robot.getSwerveDrive(), driverJoystick.getHID())
        );
        driverJoystick.button(2).whileTrue(new DriveToPosition(robot.getSwerveDrive()));
    }

    @Override
    public void teleopPeriodic() {
        scheduler.run();
        if(driverJoystick.getHID().getRawButtonPressed(1)){
            robot.getSwerveDrive().resetPose(new Pose2d(robot.getSwerveDrive().getPose().getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? Rotation2d.kZero:Rotation2d.kPi));
        }
    }

    @Override
    public void testInit() {
        scheduler.cancelAll();
        robot.getSwerveDrive().setDefaultCommand(new SwerveModuleTestingCommand(robot.getSwerveDrive(), driverJoystick.getHID()));
        Command testCommand;
        try {
            testCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test Forward 6 m"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
        driverJoystick.button(2).whileTrue(testCommand);
    }

    @Override
    public void testPeriodic() {
        scheduler.run();
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
