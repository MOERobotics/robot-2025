/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveModuleCommand;
import frc.robot.commands.standardDeviation;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.LoggedRobot;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {

    Joystick driverJoystick = new Joystick(0);
    Joystick functionJoystick = new Joystick(1);
    CommandScheduler scheduler;

    RobotContainer robot = new SubMOErine();

    Command autoCommand = Commands.none();

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
        // Elevator control
        robot.getElevator().moveVertically(InchesPerSecond.of(functionJoystick.getRawAxis(2)));
        robot.getElevator().moveHorizontally(DegreesPerSecond.of(functionJoystick.getRawAxis(3)));

        // Algae collector
        if (functionJoystick.getRawButton(5)) {
            robot.getAlgaeCollector().setWheelVelocity(RadiansPerSecond.of(-1));
        }
        else if (functionJoystick.getRawButton(6)){
            robot.getAlgaeCollector().setWheelVelocity(RadiansPerSecond.of(1));
        }
        else {
            robot.getAlgaeCollector().setWheelVelocity(RadiansPerSecond.of(0));
        }
        robot.getAlgaeCollector().setArmVelocity(RadiansPerSecond.of(functionJoystick.getRawAxis(1)));

        // Coral collector
        if (functionJoystick.getRawButton(12))  {
            robot.getCoralCollector().setCoralVelocity(RPM.of(1), RPM.of(1));
        }
        else {
            robot.getCoralCollector().setCoralVelocity(RPM.of(0), RPM.of(0));
        }
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
    }

    @Override
    public void simulationPeriodic() {
    }
}
