/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.start4_place_coral_station;
import frc.robot.container.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.*;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.simulations.*;
import org.littletonrobotics.junction.LoggedRobot;
import frc.robot.commands.junk.SwerveModuleTestingCommand;

import java.util.Set;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {


    CommandJoystick driverJoystick = new CommandJoystick(0);
    CommandJoystick functionJoystick = new CommandJoystick(1);
    CommandScheduler scheduler;
    AddressableLED m_led = new AddressableLED(9);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(2);


    RobotContainer robot = new SubMOErine();
    Autos autos;
    Command autoCommand = Commands.none();
    TimeOfFlight tof_sensor_center = new TimeOfFlight(41);


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
    PowerDistribution pdh = new PowerDistribution(23, PowerDistribution.ModuleType.kRev);

    @Override
    public void robotInit() {
        SmartDashboard.putData(field);
        tof_sensor_center.setRangeOfInterest(8,8,10,10);
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



        SmartDashboard.putNumber("Distance:", tof_sensor_center.getRange());
        scheduler.run();
        functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,0);
        if (tof_sensor_center.getRange() < 300){
            SmartDashboard.putBoolean("DROP", true);
            pdh.setSwitchableChannel(true);
            //functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,1);
        }
        else {
            SmartDashboard.putBoolean("DROP", false);
            pdh.setSwitchableChannel(false);
            //functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,0);
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
        this.autoCommand = start4_place_coral_station.buildS4IL4Command(robot);
        var swerveDrive = robot.getSwerveDrive();
        Command stopCommand = swerveDrive.run(()-> { swerveDrive.drive(0,0,0);});
        stopCommand.setName("SwerveStop");
        swerveDrive.setDefaultCommand(stopCommand);
        autoCommand = autos.getSelectedAuto();
        autoCommand.schedule();


    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
//        autoCommand.cancel();

        SmartDashboard.putData("Drive Lidar Command",  Commands.run(()->robot.getSwerveDrive().drive(0,0.2,0) ).until(()->tof_sensor_center.getRange() < 300).withTimeout(5));
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
        robot.getSwerveDrive().setDefaultCommand( new SwerveControllerCommand(robot.getSwerveDrive(), driverJoystick.getHID()));

//        ((SwerveDrive)robot.getSwerveDrive()).sysIdRoutinePivotFL.dynamic(SysIdRoutine.Direction.kReverse).schedule();
    }

    @Override
    public void teleopPeriodic() {
        if(driverJoystick.getHID().getRawButton(1)){
            robot.getSwerveDrive().resetPose(new Pose2d());
        }


        scheduler.run();
        if (tof_sensor_center.getRange() < 300){


            functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,1);
        }
        else {


            functionJoystick.setRumble(GenericHID.RumbleType.kBothRumble,0);
        }

        //robot.getSwerveDrive().setDefaultCommand(robot.getSwerveDrive().drive(0,0.2,0);
//        robot.getElevator().moveVertically(FeetPerSecond.of(MathUtil.applyDeadband(functionJoystick.getRawAxis(1),0.05)*-0.5));

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
        SimulationHelpers.setUpSim(robot);
    }

    @Override
    public void simulationPeriodic() {
        SimulationHelpers.updateSimState();
    }
}
