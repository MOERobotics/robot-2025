/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.RobotElevatorSim;
import frc.robot.subsystem.SubMOErineElevator;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModuleSim;
import org.littletonrobotics.junction.LoggedRobot;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {



    Joystick driverJoystick = new Joystick(0);
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
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

        robot.getSwerveDrive().drive(
                 -driverJoystick.getRawAxis(1),
                 -driverJoystick.getRawAxis(0),
                  driverJoystick.getRawAxis(4 /*TODO: REVERT*/)
        );
        robot.getElevator().moveVertically(InchesPerSecond.of(driverJoystick.getRawAxis(2)));
        robot.getElevator().moveHorizontally(DegreesPerSecond.of(driverJoystick.getRawAxis(3)));
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
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
