/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.LidarTest;
import frc.robot.container.FortissiMOEContainer;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.container.SwerveBotContainer;
import org.littletonrobotics.junction.LoggedRobot;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {



    Joystick driverJoystick = new Joystick(0);
    CommandScheduler scheduler;


    RobotContainer robot = new SwerveBotContainer();



    Command autoCommand = Commands.none();


    double center_dist;
    double right_dist;
    double left_dist;

    TimeOfFlight tof_center = new TimeOfFlight(42);
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

        CommandScheduler.getInstance().run();


//        if (tof_sensor_center.getRange() < 500 && tof_sensor_left.getRange() > 40 &&){
//            SmartDashboard.putString("Direction", "GOOD");
//            else if (tof_sensor_center > 500 &&)
            // }
            //  else if (lidar1distance < 40 && lidar3distance < 40){
            //   Serial.println("YES");
            // }
            //  else if (lidar2distance < 40 && lidar3distance < 40){
            //   Serial.println("YES");
            // }
            // else {
            //     Serial.println("NONONONO");
            // }


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

        SmartDashboard.putData("lidar test", new LidarTest(robot.getSwerveDrive()));
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
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
