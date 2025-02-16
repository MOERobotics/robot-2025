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
import frc.robot.commands.CoralHeadTeleopCollectCommand;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.RobotElevatorSim;
import frc.robot.subsystem.SubMOErineElevator;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModuleSim;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.commands.SwerveModuleCommand;

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


        double joyX =  -driverJoystick.getRawAxis(1);
        if(Math.abs(joyX) < 0.05 ) joyX=0;


        double joyY = -driverJoystick.getRawAxis(0);
        if(Math.abs(joyY) < 0.05 ) joyY=0;

        double joyZ = driverJoystick.getRawAxis(2);
        if(Math.abs(joyZ) < 0.05 ) joyZ=0;

        robot.getSwerveDrive().drive(
                joyX,
               joyY,
                joyZ
        );

        //CommandScheduler.getInstance().schedule(new CoralHeadTeleopCollectCommand(robot.getCoralCollector(), functionJoystick));








  double elevatorVertPower=0;

        if(driverJoystick.getRawButton(1)){
            elevatorVertPower=0.3;
        }

        if(driverJoystick.getRawButton(2)){
            elevatorVertPower=-0.3;
        }



        robot.getElevator().moveVertically(InchesPerSecond.of(elevatorVertPower));

        double elevatorHorizontalPower=0;

        if(driverJoystick.getRawButton(3)){
            elevatorHorizontalPower=0.5;
        }

        if(driverJoystick.getRawButton(4)){
            elevatorHorizontalPower=-0.5;
        }

        robot.getElevator().moveHorizontally(DegreesPerSecond.of(elevatorHorizontalPower));

  double algaeCollectorPower=0;


        if(driverJoystick.getRawButton(5)){
            algaeCollectorPower=0.08;
        }

        if(driverJoystick.getRawButton(6)){
            algaeCollectorPower=-0.08;
        }

        robot.getAlgaeCollector().setArmVelocity(RPM.of(algaeCollectorPower));


        double algaeWheelPower=0;


        if(driverJoystick.getRawButton(7)){
            algaeWheelPower=0.25;
        }

        if(driverJoystick.getRawButton(8)){
            algaeWheelPower=-0.25;
        }

        robot.getAlgaeCollector().setWheelVelocity(RPM.of(algaeWheelPower));







        double coralCollectorPowerRight=0;
        double coralCollectorPowerLeft=0;


        if(functionJoystick.getRawButton(1)){
            coralCollectorPowerRight=0.5;
            coralCollectorPowerLeft=0.5;

        }

        if(functionJoystick.getRawButton(4)){
            coralCollectorPowerRight=-0.5;
            coralCollectorPowerLeft=-0.5;
        }



        if(functionJoystick.getRawButton(3)){
            coralCollectorPowerLeft=0.3;
            coralCollectorPowerRight=1;
        }

        if(functionJoystick.getRawButton(2)){
            coralCollectorPowerLeft=1;
            coralCollectorPowerRight=1;
        }

        robot.getCoralCollector().setCoralVelocity(RPM.of(coralCollectorPowerLeft), RPM.of(coralCollectorPowerRight));





        double climberPowRear=0;
        double climberPowMid=0;


        if(driverJoystick.getRawButton(9)){
            climberPowRear=1;
            climberPowMid=1;

        }

        if(driverJoystick.getRawButton(10)){
            climberPowRear=-1;
            climberPowMid=-1;

        }

        robot.getClimberMid().setClimberVelocity(RPM.of(climberPowMid));

        robot.getClimberRear().setClimberVelocity(RPM.of(climberPowRear));



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
        if(!(robot.getSwerveDrive() instanceof  SwerveDrive)||!(robot.getElevator() instanceof SubMOErineElevator))return;
        SwerveDrive swerveDrive = (SwerveDrive) robot.getSwerveDrive();
        SubMOErineElevator elevator = (SubMOErineElevator) robot.getElevator();
        swerveModuleSimFL = new SwerveModuleSim(swerveDrive.swerveModuleFL.moduleOffset,swerveDrive.swerveModuleFL.driveMotor,swerveDrive.swerveModuleFL.pivotMotor,swerveDrive.swerveModuleFL.pivotEncoder);
        swerveModuleSimFR = new SwerveModuleSim(swerveDrive.swerveModuleFR.moduleOffset,swerveDrive.swerveModuleFR.driveMotor,swerveDrive.swerveModuleFR.pivotMotor,swerveDrive.swerveModuleFR.pivotEncoder);
        swerveModuleSimBR = new SwerveModuleSim(swerveDrive.swerveModuleBR.moduleOffset,swerveDrive.swerveModuleBR.driveMotor,swerveDrive.swerveModuleBR.pivotMotor,swerveDrive.swerveModuleBR.pivotEncoder);
        swerveModuleSimBL = new SwerveModuleSim(swerveDrive.swerveModuleBL.moduleOffset,swerveDrive.swerveModuleBL.driveMotor,swerveDrive.swerveModuleBL.pivotMotor,swerveDrive.swerveModuleBL.pivotEncoder);
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
