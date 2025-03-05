package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> implements SwerveDriveControl {
    public SwerveModule swerveModuleFL;
    public SwerveModule swerveModuleFR;
    public SwerveModule swerveModuleBL;
    public SwerveModule swerveModuleBR;
    public SwerveModule[] swerveModules;

    public @Getter SwerveDriveKinematics kinematics;
    public @Getter SwerveDriveOdometry odometry;
    public Pigeon2 pigeon;
    public Field2d PathPlannerField = new Field2d();

    public DigitalInput climbSensor = new DigitalInput(3);


    public SwerveDrive(
            SwerveModule SwerveModuleFL,
            SwerveModule SwerveModuleFR,
            SwerveModule SwerveModuleBR,
            SwerveModule SwerveModuleBL,
            Pigeon2 pigeon
    ) {

        super(new SwerveDriveInputsAutoLogged());
        this.swerveModuleBR = SwerveModuleBR;
        this.swerveModuleBL = SwerveModuleBL;
        this.swerveModuleFR = SwerveModuleFR;
        this.swerveModuleFL = SwerveModuleFL;
        this.swerveModules = new SwerveModule[]{swerveModuleFL,swerveModuleFR,swerveModuleBR,swerveModuleBL};
        this.pigeon = pigeon;
        kinematics = new SwerveDriveKinematics(
                new Translation2d(SwerveModuleFL.xPos, SwerveModuleFL.yPos),
                new Translation2d(SwerveModuleFR.xPos, SwerveModuleFR.yPos),
                new Translation2d(SwerveModuleBR.xPos, SwerveModuleBR.yPos),
                new Translation2d(SwerveModuleBL.xPos, SwerveModuleBL.yPos)
        );
        this.odometry = new SwerveDriveOdometry(
                this.kinematics,
                this.pigeon.getRotation2d(),
                new SwerveModulePosition[]{
                        this.swerveModuleFL.getModulePosition(),
                        this.swerveModuleFR.getModulePosition(),
                        this.swerveModuleBR.getModulePosition(),
                        this.swerveModuleBL.getModulePosition(),
                }
        );
        getSensors().moduleStates = new SwerveModuleState[4];
        getSensors().modulePositions = new SwerveModulePosition[4];

        getSensors().swerveModuleFL = swerveModuleFL.getSensors();
        getSensors().swerveModuleFR = swerveModuleFR.getSensors();
        getSensors().swerveModuleBR = swerveModuleBR.getSensors();
        getSensors().swerveModuleBL = swerveModuleBL.getSensors();


        SmartDashboard.putData("PPField",PathPlannerField);
        PathPlannerLogging.setLogActivePathCallback(path -> {
            PathPlannerField.getObject("traj").setPoses(path);
        });
        PathPlannerLogging.setLogCurrentPoseCallback(pose -> {
            PathPlannerField.getRobotObject().setPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback(target -> {
            PathPlannerField.getObject("target").setPose(target);
        });
    }

    @Override
    public void readSensors(SwerveDriveInputsAutoLogged sensors) {
        sensors.currentRotationRadians = this.pigeon.getRotation2d().getRadians();
        sensors.pose = this.odometry.update(
            this.pigeon.getRotation2d(),
            new SwerveModulePosition[]{
                this.swerveModuleFL.getModulePosition(),
                this.swerveModuleFR.getModulePosition(),
                this.swerveModuleBR.getModulePosition(),
                this.swerveModuleBL.getModulePosition()
            }
        );

        sensors.moduleStates[0] = swerveModuleFL.getModuleState();
        sensors.moduleStates[1] = swerveModuleFR.getModuleState();
        sensors.moduleStates[2] = swerveModuleBR.getModuleState();
        sensors.moduleStates[3] = swerveModuleBL.getModuleState();

        sensors.modulePositions[0] = swerveModuleFL.getModulePosition();
        sensors.modulePositions[1] = swerveModuleFR.getModulePosition();
        sensors.modulePositions[2] = swerveModuleBR.getModulePosition();
        sensors.modulePositions[3] = swerveModuleBL.getModulePosition();

        sensors.robotRelativeSpeeds = kinematics.toChassisSpeeds(
                swerveModuleFL.getModuleState(),
                swerveModuleFR.getModuleState(),
                swerveModuleBR.getModuleState(),
                swerveModuleBL.getModuleState());

        sensors.canClimb = !climbSensor.get();


    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rotation, boolean robotCentric) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,rotation);
        if(!robotCentric){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,getPose().getRotation());
        }
        drive(chassisSpeeds);
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        Logger.recordOutput("ChassisSpeeds", speeds);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModuleFL.setModuleState(moduleStates[0]);
        swerveModuleFR.setModuleState(moduleStates[1]);
        swerveModuleBR.setModuleState(moduleStates[2]);
        swerveModuleBL.setModuleState(moduleStates[3]);
    }




}
