/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralHeadTeleopCommand;
import frc.robot.commands.SwerveModuleCommand;
import frc.robot.container.RobotContainer;
import frc.robot.container.SubMOErine;
import frc.robot.subsystem.*;
import org.littletonrobotics.junction.LoggedRobot;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {


    Joystick driverJoystick = new Joystick(0);
    Joystick functionJoystick = new Joystick(1);
    CommandScheduler scheduler;

    RobotContainer robot = new SubMOErine();

    Command autoCommand = Commands.none();

    SwerveModuleSim swerveModuleSimFL, swerveModuleSimFR, swerveModuleSimBR, swerveModuleSimBL;
    RobotElevatorSim elevatorSim;
    AlgaeCollectorSim algaeCollectorSim;
    CoralCollectorSim coralCollectorSim;
    ClimberSim climberMidSim, climberRearSim;

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
        new CoralHeadTeleopCommand(robot.getCoralCollector(), functionJoystick).schedule();
    }

    @Override
    public void teleopPeriodic() {

        robot.getSwerveDrive().drive(
            -driverJoystick.getRawAxis(1),
            -driverJoystick.getRawAxis(0),
            driverJoystick.getRawAxis(2)
        );


        double elevatorVertPower = 0;

        if (driverJoystick.getRawButton(1)) {
            elevatorVertPower = 0.5;
        }

        if (driverJoystick.getRawButton(2)) {
            elevatorVertPower = -0.5;
        }


        robot.getElevator().moveVertically(InchesPerSecond.of(elevatorVertPower));


        double climberPowerMid = 0;

        if (driverJoystick.getRawButton(1)) {
            climberPowerMid = 0.5;
        }


        if (driverJoystick.getRawButton(2)) {
            climberPowerMid = -0.5;
        }

        robot.getClimberMid().setClimberVelocity(RPM.of(climberPowerMid));

        double climberPowerRear = 0;

        if (driverJoystick.getRawButton(1)) {
            climberPowerRear = 0.5;
        }


        if (driverJoystick.getRawButton(2)) {
            climberPowerRear = -0.5;
        }

        robot.getClimberRear().setClimberVelocity(RPM.of(climberPowerMid));


        double elevatorHorizontalPower = 0;

        if (driverJoystick.getRawButton(3)) {
            elevatorHorizontalPower = 0.5;
        }

        if (driverJoystick.getRawButton(4)) {
            elevatorHorizontalPower = -0.5;
        }

        robot.getElevator().moveHorizontally(DegreesPerSecond.of(elevatorHorizontalPower));


        double algaeCollectorPower = 0;


        if (driverJoystick.getRawButton(7)) {
            algaeCollectorPower = 0.5;
        }

        if (driverJoystick.getRawButton(8)) {
            algaeCollectorPower = -0.5;
        }

        robot.getAlgaeCollector().setArmVelocity(RPM.of(algaeCollectorPower));


        double algaeWheelPower = 0;


        if (driverJoystick.getRawButton(9)) {
            algaeWheelPower = 0.5;
        }

        if (driverJoystick.getRawButton(10)) {
            algaeWheelPower = -0.5;
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
        if (!(robot.getSwerveDrive() instanceof SwerveDrive)
            || !(robot.getElevator() instanceof SubMOErineElevator)
            || !(robot.getAlgaeCollector() instanceof AlgaeCollector)
            || !(robot.getCoralCollector() instanceof CoralCollector)
            || !(robot.getClimberRear() instanceof  SubMOErineClimber)
            || !(robot.getClimberMid() instanceof SubMOErineClimber)) return;
        SwerveDrive swerveDrive = (SwerveDrive) robot.getSwerveDrive();
        SubMOErineElevator elevator = (SubMOErineElevator) robot.getElevator();
        AlgaeCollector algaeCollector = (AlgaeCollector) robot.getAlgaeCollector();
        CoralCollector coralCollector = (CoralCollector) robot.getCoralCollector();
        SubMOErineClimber climberMid = (SubMOErineClimber) robot.getClimberMid();
        SubMOErineClimber climberRear = (SubMOErineClimber) robot.getClimberRear();
        swerveModuleSimFL = new SwerveModuleSim(swerveDrive.swerveModuleFL.heading, swerveDrive.swerveModuleFL.driveMotor, swerveDrive.swerveModuleFL.pivotMotor, swerveDrive.swerveModuleFL.compass);
        swerveModuleSimFR = new SwerveModuleSim(swerveDrive.swerveModuleFR.heading, swerveDrive.swerveModuleFR.driveMotor, swerveDrive.swerveModuleFR.pivotMotor, swerveDrive.swerveModuleFR.compass);
        swerveModuleSimBR = new SwerveModuleSim(swerveDrive.swerveModuleBR.heading, swerveDrive.swerveModuleBR.driveMotor, swerveDrive.swerveModuleBR.pivotMotor, swerveDrive.swerveModuleBR.compass);
        swerveModuleSimBL = new SwerveModuleSim(swerveDrive.swerveModuleBL.heading, swerveDrive.swerveModuleBL.driveMotor, swerveDrive.swerveModuleBL.pivotMotor, swerveDrive.swerveModuleBL.compass);
        elevatorSim = new RobotElevatorSim(elevator);
        algaeCollectorSim = new AlgaeCollectorSim(algaeCollector);
        coralCollectorSim = new CoralCollectorSim(coralCollector);
        climberMidSim = new ClimberSim(climberMid);
        climberRearSim = new ClimberSim(climberRear);
    }

    @Override
    public void simulationPeriodic() {
        swerveModuleSimFL.updateSimState();
        swerveModuleSimFR.updateSimState();
        swerveModuleSimBR.updateSimState();
        swerveModuleSimBL.updateSimState();
        elevatorSim.updteSimState();
        algaeCollectorSim.updateSimState();
        coralCollectorSim.updateSimState();
        climberMidSim.updateSimState();
        climberRearSim.updateSimState();
    }
}
