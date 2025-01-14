/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;
import org.littletonrobotics.junction.LoggedRobot;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {



    Joystick driverJoystick = new Joystick(0);
    CommandScheduler scheduler;

    // RobotContainer robot;

    SwerveDrive drive = new SwerveDrive(
        new SwerveModule(//FL
            new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(16, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(34),
            Inches.of(14),
            Inches.of(14),
            Degrees.of(45)
        ),
        new SwerveModule(//FR
            new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(2, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(33),
            Inches.of(14),
            Inches.of(-14),
            Degrees.of(-45)
        ),
        new SwerveModule(//BR
            new SparkMax( 1, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(32),
            Inches.of(-14),
            Inches.of(-14),
            Degrees.of(-135)
        ),
        new SwerveModule(//BL
            new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(31),
            Inches.of(-14),
            Inches.of(14),
            Degrees.of(135)
        ),
        new Pigeon2(0)
    );

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
        Joystick asdf = new Joystick(0);
        drive.Drive(asdf.getX(),asdf.getY(),asdf.getZ());
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
