/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.*;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {

    Joystick driverJoystick = new Joystick(0);
    Joystick functionJoystick = new Joystick(1);
    CommandScheduler scheduler;

    RobotContainer robot = new SubMOErine();

    Command autoCommand = Commands.none();

    SwerveModuleSim swerveModuleSimFL,swerveModuleSimFR,swerveModuleSimBR,swerveModuleSimBL;
    RobotElevatorSim elevatorSim;

    @Override
    public void robotInit() {

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);

        MOELogger.setupLogging(this);

        scheduler = CommandScheduler.getInstance();
        //robot.getDrive().setDefaultCommand(Commands.none());
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
        /*try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
            PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
            AutoBuilder.followPath(path).schedule();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }*/
        CommandScheduler.getInstance().cancelAll();
        new ElevatorAutoCommand(robot.getElevator(),
                3,
                MetersPerSecond.of(0.5),
                false).schedule();

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        new ElevatorTeleopCommand(
            robot.getElevator(),
            functionJoystick
        );
        new AlgaeCollectorTeleopCommand(
            robot.getAlgaeCollector(),
            functionJoystick
        ).schedule();
        new DriveCoralCollectorCommand(
            robot.getCoralCollector(),
            functionJoystick
        );
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        robot.getSwerveDrive().setDefaultCommand(new SwerveModuleCommand(robot.getSwerveDrive(), driverJoystick));

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {
        Command commandSD = new standardDeviation();
        SmartDashboard.putData("Standard Deviation", commandSD);
        commandSD.schedule();
        if(!(robot.getSwerveDrive() instanceof  SwerveDrive)||!(robot.getElevator() instanceof SubMOErineElevator))return;
        SwerveDrive swerveDrive = (SwerveDrive) robot.getSwerveDrive();
        SubMOErineElevator elevator = (SubMOErineElevator) robot.getElevator();
        swerveModuleSimFL = new SwerveModuleSim(swerveDrive.swerveModuleFL.heading,swerveDrive.swerveModuleFL.driveMotor,swerveDrive.swerveModuleFL.pivotMotor,swerveDrive.swerveModuleFL.compass);
        swerveModuleSimFR = new SwerveModuleSim(swerveDrive.swerveModuleFR.heading,swerveDrive.swerveModuleFR.driveMotor,swerveDrive.swerveModuleFR.pivotMotor,swerveDrive.swerveModuleFR.compass);
        swerveModuleSimBR = new SwerveModuleSim(swerveDrive.swerveModuleBR.heading,swerveDrive.swerveModuleBR.driveMotor,swerveDrive.swerveModuleBR.pivotMotor,swerveDrive.swerveModuleBR.compass);
        swerveModuleSimBL = new SwerveModuleSim(swerveDrive.swerveModuleBL.heading,swerveDrive.swerveModuleBL.driveMotor,swerveDrive.swerveModuleBL.pivotMotor,swerveDrive.swerveModuleBL.compass);
        elevatorSim = new RobotElevatorSim(elevator);
    }

    @Override
    public void simulationPeriodic() {
        swerveModuleSimFL.updateSimState();
        swerveModuleSimFR.updateSimState();
        swerveModuleSimBR.updateSimState();
        swerveModuleSimBL.updateSimState();
        elevatorSim.updteSimState();
    }
}
