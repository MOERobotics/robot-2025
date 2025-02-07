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
import frc.robot.container.FortissiMOEContainer;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.commands.SwerveModuleCommand;

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
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {

        robot.getSwerveDrive().drive(
                -driverJoystick.getRawAxis(1),
                -driverJoystick.getRawAxis(0),
                driverJoystick.getRawAxis(2)
        );

        double climberPower=0;

        if(driverJoystick.getRawButton(1)){
            climberPower=0.5;
        }



        if(driverJoystick.getRawButton(2)){
            climberPower=-0.5;
        }

        robot.getClimber().setClimberTopVelocity(RPM.of(climberPower));

        robot.getClimber().setClimberBottomVelocity(RPM.of(climberPower));




        /*
        double elevatorVertPower=0;

        if(driverJoystick.getRawButton(1)){
            elevatorVertPower=0.5;
        }

        if(driverJoystick.getRawButton(2)){
            elevatorVertPower=-0.5;
        }



        robot.getElevator().moveVertically(InchesPerSecond.of(elevatorVertPower));


         */

        double elevatorHorizontalPower=0;

        if(driverJoystick.getRawButton(3)){
            elevatorHorizontalPower=0.5;
        }

        if(driverJoystick.getRawButton(4)){
            elevatorHorizontalPower=-0.5;
        }

        robot.getElevator().moveHorizontally(DegreesPerSecond.of(elevatorHorizontalPower));


        double coralCollectorPowerRight=0;
        double coralCollectorPowerLeft=0;


        if(driverJoystick.getRawButton(5)){
            coralCollectorPowerRight=0.5;
            coralCollectorPowerLeft=0.5;

        }

        if(driverJoystick.getRawButton(6)){
            coralCollectorPowerRight=-0.5;
            coralCollectorPowerLeft=-0.5;
        }

        robot.getCoralCollector().setCoralVelocity(RPM.of(coralCollectorPowerLeft), RPM.of(coralCollectorPowerRight));


        double algaeCollectorPower=0;


        if(driverJoystick.getRawButton(7)){
            algaeCollectorPower=0.5;
        }

        if(driverJoystick.getRawButton(8)){
            algaeCollectorPower=-0.5;
        }

        robot.getAlgaeCollector().setArmVelocity(RPM.of(algaeCollectorPower));


        double algaeWheelPower=0;


        if(driverJoystick.getRawButton(9)){
            algaeWheelPower=0.5;
        }

        if(driverJoystick.getRawButton(10)){
            algaeWheelPower=-0.5;
        }

        robot.getAlgaeCollector().setWheelVelocity(RPM.of(algaeWheelPower));

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
    }

    @Override
    public void simulationPeriodic() {
    }
}
