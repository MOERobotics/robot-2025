/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.container.*;
import frc.robot.subsystem.*;
import org.littletonrobotics.junction.LoggedRobot;

import java.util.Set;

import static edu.wpi.first.units.Units.*;


public class Robot extends LoggedRobot {


    CommandJoystick driverJoystick = new CommandJoystick(0);
    Joystick functionJoystick = new Joystick(1);
    CommandScheduler scheduler;


    RobotContainer robot = new FortissiMOEContainer();
    Autos autos;
    Command autoCommand = Commands.none();


    SendableChooser<SwerveDriveControl.CommandType> chooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.ModuleType> modChooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.DriveOrPivot> driveTypeChooser = new SendableChooser<>();
    SendableChooser<Command> pathChooser = new SendableChooser<>();
    Field2d field = new Field2d();

    SwerveModuleSim swerveModuleSimFL, swerveModuleSimFR, swerveModuleSimBR, swerveModuleSimBL;
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
//        if (tof_sensor_center.getRange() < 254){
//            SmartDashboard.putString("DROP", "YES");
//        }
//        else {
//            SmartDashboard.putString("DROP", "NO");
//        }
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
        autoCommand.cancel();
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
            double driveX = -driverJoystick.getRawAxis(0);
            double driveY = driverJoystick.getRawAxis(1);
            double driveTheta = -driverJoystick.getRawAxis(4);
            driveTheta = MathUtil.applyDeadband(driveTheta, 0.2, 1);
            driveX = MathUtil.applyDeadband(driveX, 0.1, 1);
            driveY = MathUtil.applyDeadband(driveY, 0.1, 1);
            robot.getSwerveDrive().drive(
                    2*driveX,
                    2*driveY,
                    Math.PI*driveTheta //TODO: REVERT
            );
        }));
//        Command Drive = new driveToPosition((SwerveDrive) robot.getSwerveDrive(), DriveToTag.getClosestTarget(robot.getSwerveDrive().getPose()));
//        SmartDashboard.putData(Drive);

        Command DriveTraj = new driveToPosition2(robot.getSwerveDrive());
        SmartDashboard.putData(DriveTraj);
//        ((SwerveDrive)robot.getSwerveDrive()).sysIdRoutinePivotFL.dynamic(SysIdRoutine.Direction.kReverse).schedule();

    }

    @Override
    public void teleopPeriodic() {
        if(driverJoystick.getHID().getRawButton(2)){
            robot.getSwerveDrive().resetPose(new Pose2d(7.566, 6.135, Rotation2d.fromDegrees(180)));
        }



        /*double elevatorVertPower = 0;

        if (driverJoystick.getHID().getRawButton(1)) {
            elevatorVertPower = 0.5;
        }

        if (driverJoystick.getHID().getRawButton(2)) {
            elevatorVertPower = -0.5;
        }


        robot.getElevator().moveVertically(InchesPerSecond.of(elevatorVertPower));


        double climberPowerMid = 0;

        if (driverJoystick.getHID().getRawButton(1)) {
            climberPowerMid = 0.5;
        }


        if (driverJoystick.getHID().getRawButton(2)) {
            climberPowerMid = -0.5;
        }

        robot.getClimberMid().setClimberVelocity(RPM.of(climberPowerMid));

        double climberPowerRear = 0;

        if (driverJoystick.getHID().getRawButton(1)) {
            climberPowerRear = 0.5;
        }


        if (driverJoystick.getHID().getRawButton(2)) {
            climberPowerRear = -0.5;
        }

        robot.getClimberRear().setClimberVelocity(RPM.of(climberPowerMid));


        double elevatorHorizontalPower = 0;

        if (driverJoystick.getHID().getRawButton(3)) {
            elevatorHorizontalPower = 0.5;
        }

        if (driverJoystick.getHID().getRawButton(4)) {
            elevatorHorizontalPower = -0.5;
        }

        robot.getElevator().moveHorizontally(DegreesPerSecond.of(elevatorHorizontalPower));


        double algaeCollectorPower = 0;


        if (driverJoystick.getHID().getRawButton(7)) {
            algaeCollectorPower = 0.5;
        }

        if (driverJoystick.getHID().getRawButton(8)) {
            algaeCollectorPower = -0.5;
        }

        robot.getAlgaeCollector().setArmVelocity(RPM.of(algaeCollectorPower));


        double algaeWheelPower = 0;


        if (driverJoystick.getHID().getRawButton(9)) {
            algaeWheelPower = 0.5;
        }

        if (driverJoystick.getHID().getRawButton(10)) {
            algaeWheelPower = -0.5;
        }

        robot.getAlgaeCollector().setWheelVelocity(RPM.of(algaeWheelPower));*/

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        robot.getSwerveDrive().setDefaultCommand(new SwerveModuleCommand(robot.getSwerveDrive(), driverJoystick.getHID()));

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
        swerveModuleSimFL = new SwerveModuleSim(swerveDrive.swerveModuleFL.offset, swerveDrive.swerveModuleFL.driveMotor, swerveDrive.swerveModuleFL.pivotMotor, swerveDrive.swerveModuleFL.pivotEncoder);
        swerveModuleSimFR = new SwerveModuleSim(swerveDrive.swerveModuleFR.offset, swerveDrive.swerveModuleFR.driveMotor, swerveDrive.swerveModuleFR.pivotMotor, swerveDrive.swerveModuleFR.pivotEncoder);
        swerveModuleSimBR = new SwerveModuleSim(swerveDrive.swerveModuleBR.offset, swerveDrive.swerveModuleBR.driveMotor, swerveDrive.swerveModuleBR.pivotMotor, swerveDrive.swerveModuleBR.pivotEncoder);
        swerveModuleSimBL = new SwerveModuleSim(swerveDrive.swerveModuleBL.offset, swerveDrive.swerveModuleBL.driveMotor, swerveDrive.swerveModuleBL.pivotMotor, swerveDrive.swerveModuleBL.pivotEncoder);
        elevatorSim = new RobotElevatorSim(elevator);
        algaeCollectorSim = new AlgaeCollectorSim(algaeCollector);
        coralCollectorSim = new CoralCollectorSim(coralCollector);
        climberMidSim = new ClimberSim(climberMid);
        climberRearSim = new ClimberSim(climberRear);
    }

    @Override
    public void simulationPeriodic() {
        return;
        /*swerveModuleSimFL.updateSimState();
        swerveModuleSimFR.updateSimState();
        swerveModuleSimBR.updateSimState();
        swerveModuleSimBL.updateSimState();
        elevatorSim.updteSimState();
        algaeCollectorSim.updateSimState();
        coralCollectorSim.updateSimState();
        climberMidSim.updateSimState();
        climberRearSim.updateSimState();*/
    }
}
