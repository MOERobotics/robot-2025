package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import frc.robot.utils.LimelightHelpers;
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
    public @Getter SwerveDrivePoseEstimator odometry;
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
        this.odometry = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.pigeon.getRotation2d(),
                new SwerveModulePosition[]{
                        this.swerveModuleFL.getModulePosition(),
                        this.swerveModuleFR.getModulePosition(),
                        this.swerveModuleBR.getModulePosition(),
                        this.swerveModuleBL.getModulePosition(),
                },
                new Pose2d()
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
        sensors.currentRotationRadians = this.odometry.getEstimatedPosition().getRotation().getRadians();

        LimelightHelpers.SetRobotOrientation("limelight",odometry.getEstimatedPosition().getRotation().getDegrees(), 0,0,0,0,0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        boolean rejectUpdate = false;
        if (pigeon.getAngularVelocityZWorld().getValue().abs(DegreesPerSecond)>=360){
            rejectUpdate = true;
        }
        if (limelightMeasurement.tagCount == 0){
            rejectUpdate = true;
        }
        if(!rejectUpdate){
            this.odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,999999));
            this.odometry.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
        Logger.recordOutput("Vision Update", rejectUpdate);
        Logger.recordOutput("LLPose", limelightMeasurement.pose);

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
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,4);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModuleFL.setModuleState(moduleStates[0]);
        swerveModuleFR.setModuleState(moduleStates[1]);
        swerveModuleBR.setModuleState(moduleStates[2]);
        swerveModuleBL.setModuleState(moduleStates[3]);
    }




}
