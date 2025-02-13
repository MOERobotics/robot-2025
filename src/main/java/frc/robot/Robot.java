/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.container.FortissiMOEContainer;
import frc.robot.container.SubMOErine;
import frc.robot.container.RobotContainer;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveDriveControl;
import org.littletonrobotics.junction.LoggedRobot;

import java.util.Set;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {



    CommandJoystick driverJoystick = new CommandJoystick(1);
    CommandScheduler scheduler;


    RobotContainer robot = new FortissiMOEContainer();



    Command autoCommand = Commands.none();
//    TimeOfFlight tof_sensor_center = new TimeOfFlight(42);


    SendableChooser<SwerveDriveControl.CommandType> chooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.ModuleType> modChooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.DriveOrPivot> driveTypeChooser = new SendableChooser<>();
    SendableChooser<Command> pathChooser = new SendableChooser<>();
    Field2d field = new Field2d();

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

        PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
        try {
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
        }

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
        robot.getSwerveDrive().resetPose(new Pose2d());
        try {
            // Load the path you want to follow using its name in the GUI
//            PathPlannerAutoBuilder.configure(robot.getSwerveDrive());
//            PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
//            AutoBuilder.followPath(path).schedule();
            pathChooser.getSelected().schedule();
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
                    driveX,
                    driveY,
                    driveTheta //TODO: REVERT
            );
        }));

//        ((SwerveDrive)robot.getSwerveDrive()).sysIdRoutinePivotFL.dynamic(SysIdRoutine.Direction.kReverse).schedule();
    }

    @Override
    public void teleopPeriodic() {



            double elevatorPowerVert = 0;
            if (driverJoystick.getHID().getRawButton(1)) {
                elevatorPowerVert = 0.5;
            }
            if (driverJoystick.getHID().getRawButton(2)) {
                elevatorPowerVert = -0.5;
            }
        /*    if(driverJoystick.getHID().getRawButton(3)){
                robot.getSwerveDrive().pivotAngle(Radians.of(Math.atan2(-driverJoystick.getRawAxis(0),-driverJoystick.getRawAxis(1))));
            } */
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
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    @Override
    public void simulationPeriodic() {
    }
}
