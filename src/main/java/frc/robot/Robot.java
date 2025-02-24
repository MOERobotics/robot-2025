/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.start2_place_coral_station;
import frc.robot.container.*;
import frc.robot.commands.*;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.*;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.simulations.*;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.commands.junk.SwerveModuleTestingCommand;

import java.util.Set;


public class Robot extends LoggedRobot {


    CommandJoystick driverJoystick = new CommandJoystick(0);
    CommandJoystick functionJoystick = new CommandJoystick(1);
    CommandScheduler scheduler;


    RobotContainer robot = new SubMOErine();
    Autos autos;
    Command autoCommand = Commands.none();
    TimeOfFlight tof_sensor_center = new TimeOfFlight(42);


    SendableChooser<SwerveDriveControl.CommandType> chooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.ModuleType> modChooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.DriveOrPivot> driveTypeChooser = new SendableChooser<>();
    SendableChooser<Command> pathChooser = new SendableChooser<>();
    Field2d field = new Field2d();

    SwerveModuleSim swerveModuleSimFL,swerveModuleSimFR,swerveModuleSimBR,swerveModuleSimBL;
    RobotElevatorSim elevatorSim;
    AlgaeCollectorSim algaeCollectorSim;
    CoralCollectorSim coralCollectorSim;
    ClimberSim climberMidSim, climberRearSim;

    @Override
    public void robotInit() {
        SmartDashboard.putData(field);
        PathPlannerLogging.setLogActivePathCallback(path -> {
            field.getObject("traj").setPoses(path);
        });
        PathPlannerLogging.setLogCurrentPoseCallback(path -> {
            field.getRobotObject().setPose(path);
        });
        PathPlannerLogging.setLogTargetPoseCallback(path -> {
            field.getObject("target").setPose(path);
        });

        if (isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);

        MOELogger.setupLogging(this);

        scheduler = CommandScheduler.getInstance();
        //robot.getDrive().setDefaultCommand(Commands.none());

        chooser.addOption("SysTDStatic_Forward",SwerveDriveControl.CommandType.QuasistaticForward);
        chooser.addOption("SysTDStatic_Reverse", SwerveDriveControl.CommandType.QuasistaticReverse);
        chooser.addOption("SysTDDynamic_Forward", SwerveDriveControl.CommandType.DynamicForward);
        chooser.setDefaultOption("SysTDDynamic_Reverse", SwerveDriveControl.CommandType.DynamicReverse);
        SmartDashboard.putData("Command Chooser",chooser);

        modChooser.addOption("FL", SwerveDriveControl.ModuleType.modFL);
        modChooser.addOption("FR", SwerveDriveControl.ModuleType.modFR);
        modChooser.addOption("BL", SwerveDriveControl.ModuleType.modBL);
        modChooser.setDefaultOption("BR", SwerveDriveControl.ModuleType.modBR);
        SmartDashboard.putData("Module Chooser",modChooser);

        driveTypeChooser.setDefaultOption("Pivot", SwerveDriveControl.DriveOrPivot.setPivot);
        driveTypeChooser.addOption("Drive", SwerveDriveControl.DriveOrPivot.setDrive);
        SmartDashboard.putData("DriveTypeChooser",driveTypeChooser);
        SmartDashboard.putData("Scheduler", scheduler);

        autos = new Autos(robot);

        this.autoCommand = start2_place_coral_station.buildS2EL4Command(robot);
        /*try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("Test-Forward 5 Ft");
            PathPlannerPath path2 = PathPlannerPath.fromPathFile("Test-Forward-Left 5 Ft");
            PathPlannerPath path3 = PathPlannerPath.fromPathFile("Test-Arc 5 Ft");
            pathChooser.setDefaultOption("Test Path - Forward",AutoBuilder.followPath(path1));
            pathChooser.addOption("Test Path - Forward-Left",AutoBuilder.followPath(path2));
            pathChooser.addOption("Test Path - Arc",AutoBuilder.followPath(path3));
            SmartDashboard.putData("Path Chooser",pathChooser);
        }catch (Exception e){
            e.printStackTrace();
            throw new RuntimeException(e);
        }*/

    }


    @Override
    public void robotPeriodic() {
        scheduler.run();
        if (tof_sensor_center.getRange() < 254){
            SmartDashboard.putString("DROP", "YES");
        }
        else {
            SmartDashboard.putString("DROP", "NO");
        }
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
        autoCommand.schedule();

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        autoCommand.cancel();
        new ElevatorTeleopCommand(
            robot.getElevator(),
            functionJoystick.getHID()
        ).schedule();
        new AlgaeCollectorTeleopCommand(
            robot.getAlgaeCollector(),
            functionJoystick.getHID()
        ).schedule();
        new CoralHeadTeleopCommand(
            robot.getCoralHead(),
            functionJoystick.getHID(),
            robot.getElevator()
        ).schedule();
        new ClimberTeleopCommand(
            robot.getClimberRear(),
            robot.getClimberMid(),
            driverJoystick.getHID()
        ).schedule();
        var driveOneSecond = robot.getSwerveDrive().run(() ->
        {
            robot.getSwerveDrive().drive(1, 0, 0);
        }).withTimeout(1)
                .withName("DriveOneSec");
        driverJoystick.button(4).whileTrue(Commands.defer(() -> {

            if (chooser.getSelected() == SwerveDriveControl.CommandType.QuasistaticForward) {
                return  ((SwerveDrive) robot.getSwerveDrive()).sysIdRoutineDrive.quasistatic(SysIdRoutine.Direction.kForward);
            }
            if (chooser.getSelected() == SwerveDriveControl.CommandType.QuasistaticReverse) {
                return ((SwerveDrive) robot.getSwerveDrive()).sysIdRoutineDrive.quasistatic(SysIdRoutine.Direction.kReverse);
            }
            if (chooser.getSelected() == SwerveDriveControl.CommandType.DynamicForward) {
                return ((SwerveDrive) robot.getSwerveDrive()).sysIdRoutineDrive.dynamic(SysIdRoutine.Direction.kForward);
            }
            if (chooser.getSelected() == SwerveDriveControl.CommandType.DynamicReverse) {
                return ((SwerveDrive) robot.getSwerveDrive()).sysIdRoutineDrive.dynamic(SysIdRoutine.Direction.kReverse);
            }
            return Commands.none();
        }, Set.of(robot.getSwerveDrive())));
        driverJoystick.button(5).whileTrue(driveOneSecond);
        robot.getSwerveDrive().setDefaultCommand(robot.getSwerveDrive().run(() -> {
            double driveX =  -driverJoystick.getRawAxis(1);
            double driveY = -driverJoystick.getRawAxis(0);
            double driveTheta = -driverJoystick.getRawAxis(2);
            driveTheta = MathUtil.applyDeadband(driveTheta, 0.2, 1);
            driveX = MathUtil.applyDeadband(driveX, 0.1, 1);
            driveY = MathUtil.applyDeadband(driveY, 0.1, 1);
            robot.getSwerveDrive().drive(
                    2*driveX,
                    2*driveY,
                    Math.PI*driveTheta //TODO: REVERT
            );
        }));

//        ((SwerveDrive)robot.getSwerveDrive()).sysIdRoutinePivotFL.dynamic(SysIdRoutine.Direction.kReverse).schedule();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        robot.getSwerveDrive().setDefaultCommand(new SwerveModuleTestingCommand(robot.getSwerveDrive(), driverJoystick.getHID()));

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

        Command commandSD = new standardDeviation();
        SmartDashboard.putData("Standard Deviation", commandSD);
        commandSD.schedule();
        if (!(robot.getSwerveDrive() instanceof SwerveDrive)
            || !(robot.getElevator() instanceof SubMOErineElevator)
            || !(robot.getAlgaeCollector() instanceof AlgaeCollector)
            || !(robot.getCoralHead() instanceof CoralHead)
            || !(robot.getClimberRear() instanceof SubMOErineClimber)
            || !(robot.getClimberMid() instanceof SubMOErineClimber)) return;
        SwerveDrive swerveDrive = (SwerveDrive) robot.getSwerveDrive();
        SubMOErineElevator elevator = (SubMOErineElevator) robot.getElevator();
        swerveModuleSimFL = new SwerveModuleSim(swerveDrive.swerveModuleFL.moduleOffset,swerveDrive.swerveModuleFL.driveMotor,swerveDrive.swerveModuleFL.pivotMotor,swerveDrive.swerveModuleFL.pivotEncoder);
        swerveModuleSimFR = new SwerveModuleSim(swerveDrive.swerveModuleFR.moduleOffset,swerveDrive.swerveModuleFR.driveMotor,swerveDrive.swerveModuleFR.pivotMotor,swerveDrive.swerveModuleFR.pivotEncoder);
        swerveModuleSimBR = new SwerveModuleSim(swerveDrive.swerveModuleBR.moduleOffset,swerveDrive.swerveModuleBR.driveMotor,swerveDrive.swerveModuleBR.pivotMotor,swerveDrive.swerveModuleBR.pivotEncoder);
        swerveModuleSimBL = new SwerveModuleSim(swerveDrive.swerveModuleBL.moduleOffset,swerveDrive.swerveModuleBL.driveMotor,swerveDrive.swerveModuleBL.pivotMotor,swerveDrive.swerveModuleBL.pivotEncoder);
        elevatorSim = new RobotElevatorSim(elevator);
        algaeCollectorSim = new AlgaeCollectorSim((AlgaeCollector)robot.getAlgaeCollector());
        coralCollectorSim = new CoralCollectorSim((CoralHead)robot.getCoralHead());
        climberMidSim = new ClimberSim(robot.getClimberMid());
        climberRearSim = new ClimberSim(robot.getClimberRear());


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
