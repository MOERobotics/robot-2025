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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.container.FortissiMOEContainer;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.SwerveDrive;
import org.littletonrobotics.junction.LoggedRobot;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {



    Joystick driverJoystick = new Joystick(0);
    CommandScheduler scheduler;


    RobotContainer robot = new FortissiMOEContainer();



    Command autoCommand = Commands.none();
//    TimeOfFlight tof_sensor_center = new TimeOfFlight(42);

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
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
            PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
            AutoBuilder.followPath(path).schedule();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

        /*robot.getSwerveDrive().drive(
                 -driverJoystick.getRawAxis(1),
                 -driverJoystick.getRawAxis(0),
                  driverJoystick.getRawAxis(2 /*TODO: REVERT)
        );*/
        double elevatorPowerVert = 0;
        if (driverJoystick.getRawButton(1)){
            elevatorPowerVert = 0.5;
        }
        if (driverJoystick.getRawButton(2)){
            elevatorPowerVert = -0.5;
        }
        robot.getElevator().moveVertically(InchesPerSecond.of(elevatorPowerVert));
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
    }

    @Override
    public void simulationPeriodic() {
    }
}
