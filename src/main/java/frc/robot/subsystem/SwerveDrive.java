package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import lombok.Getter;
import org.ejml.equation.Variable;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

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
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public PIDController xController = new PIDController(5.0, 0.0, 0.0);
    public PIDController yController = new PIDController(5.0, 0.0, 0.0);
    public ProfiledPIDController thetaController = new ProfiledPIDController(5.0, 0.0, 0e-4,  new TrapezoidProfile.Constraints(0.5, 0.5));


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


        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.pigeon.getRotation2d(),
                new SwerveModulePosition[]{
                        this.swerveModuleFL.getModulePosition(),
                        this.swerveModuleFR.getModulePosition(),
                        this.swerveModuleBR.getModulePosition(),
                        this.swerveModuleBL.getModulePosition(),
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, 0.05),
                VecBuilder.fill(0.5, 0.5, 0.5)
        );


        getSensors().moduleStates = new SwerveModuleState[4];
        getSensors().modulePositions = new SwerveModulePosition[4];

        getSensors().swerveModuleFL = swerveModuleFL.getSensors();
        getSensors().swerveModuleFR = swerveModuleFR.getSensors();
        getSensors().swerveModuleBR = swerveModuleBR.getSensors();
        getSensors().swerveModuleBL = swerveModuleBL.getSensors();


        SmartDashboard.putData(PathPlannerField);
        PathPlannerLogging.setLogActivePathCallback(path -> {
            PathPlannerField.getObject("traj").setPoses(path);
        });
        PathPlannerLogging.setLogCurrentPoseCallback(path -> {
            PathPlannerField.getRobotObject().setPose(path);
        });
        PathPlannerLogging.setLogTargetPoseCallback(path -> {
            PathPlannerField.getObject("target").setPose(path);
        });
    }

    @Override
    public Pose2d getFieldPose() {
        return poseEstimator.getEstimatedPosition();
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

        var LimeLightPose = LimelightHelpers.getBotPose3dWithTime("limelight");

        if (!LimeLightPose.pose().toPose2d().equals(Pose2d.kZero)){
            this.poseEstimator.addVisionMeasurement(LimeLightPose.pose().toPose2d(), MathSharedStore.getTimestamp());
        }

        sensors.Pose2dFromLL = LimeLightPose.pose().toPose2d();
        sensors.NewPose2dFromLL = this.poseEstimator.getEstimatedPosition();
        //sensors.NewPose2dFromLL = LimeLightPose.pose().toPose2d();



    }
   /*
   // pose 2d, float seconds,
   public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {


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

    /**
     * Drives an individual swerve module of the swerve drive
     *
     * @param index    the index of the swerve module 0-3 represent modules FL,FR,BR,BL
     */
    @Override
    public void driveSingleModule(int index, double xSpeed, double ySpeed, double rotation) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rotation);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModules[index].drive(moduleStates[index].speedMetersPerSecond);
        swerveModules[index].pivot(moduleStates[index].angle.getMeasure());
    }
    @Override
    public void resetPose(Pose2d newPose) {
        SwerveDriveControl.super.resetPose(newPose);
        this.poseEstimator.resetPose(newPose);
    }

    @Override
    public Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond) {
        TrajectoryConfig config = new TrajectoryConfig(0.1, 0.1);
        config.setEndVelocity(endVelocityMetersPerSecond);
        config.setStartVelocity(startVelocityMetersPerSecond);

        var trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            internalPoints,
            end,
            config
        );
        PathPlannerField.getObject("traj").setTrajectory(trajectory);
        SmartDashboard.putNumber("Time", trajectory.getTotalTimeSeconds());
        SmartDashboard.putNumber("trajEndRotation", trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("desiredEndRot", end.getRotation().getDegrees());
        SwerveControllerCommand trajCommand = new SwerveControllerCommand(
                trajectory,
                this::getFieldPose,
                kinematics,
                xController,
                yController,
                thetaController,
                this::setModuleStates,
                this
        );
//        field.getRobotObject().setTrajectory(trajectory);
//        SmartDashboard.putData("trajectory",field);
        Logger.recordOutput("trajectory/start", start);
        Logger.recordOutput("trajectory/goal", end);
        Logger.recordOutput(("trajectory/traj"), trajectory);
        return trajCommand;
    }




    public void setModuleStates(SwerveModuleState[] moduleStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 0.2);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModuleFL.setModuleState(moduleStates[0]);
        swerveModuleFR.setModuleState(moduleStates[1]);
        swerveModuleBR.setModuleState(moduleStates[2]);
        swerveModuleBL.setModuleState(moduleStates[3]);
    }
}
