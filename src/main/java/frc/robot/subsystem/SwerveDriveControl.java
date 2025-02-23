package frc.robot.subsystem;

import com.fasterxml.jackson.databind.introspect.Annotated;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public interface SwerveDriveControl extends Subsystem {
    /*default Pose2d getPose() {
        return this.getSensors().pose;
    }*/

    default Pose2d getPose(){
        Pose3d LimeLightPose = LimelightHelpers.getBotPose3d("limelight");
        Transform2d odometryCorrection = new Transform2d(LimeLightPose.getX(), LimeLightPose.getY(), new Rotation2d(LimeLightPose.getRotation().getAngle()));
        if (!LimeLightPose.toPose2d().equals(Pose2d.kZero)){
            return this.getSensors().pose.transformBy(odometryCorrection);
        }
        return this.getSensors().pose;
    }

    SwerveDriveInputsAutoLogged getSensors();

    default void resetPose(Pose2d newPose) {
        this.getOdometry().resetPose(newPose);
    }

    void drive(ChassisSpeeds speeds);

    default void drive(double xSpeed, double ySpeed, double rotation) {
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotation));
    }

    public static enum CommandType {
        QuasistaticForward, QuasistaticReverse, DynamicForward, DynamicReverse
    }
    public static enum ModuleType {
        modFL, modFR, modBL, modBR, allMods
    }
    public static enum DriveOrPivot {
        setDrive, setPivot,
    }

    default public Command sysIDCommandsPivot(CommandType commandType, ModuleType moduleType){
       return Commands.none();
    }
    default public Command sysIDCommandsDrive(CommandType commandType){
        return Commands.none();
    }

    void driveSingleModule (int index, double xSpeed, double ySpeed, double rotation);

    default ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getSensors().robotRelativeSpeeds;
    }

    edu.wpi.first.math.kinematics.SwerveDriveKinematics getKinematics();

    edu.wpi.first.math.kinematics.SwerveDriveOdometry getOdometry();

    public void pivotAngle(Angle angle);

    @AutoLog
    public static class SwerveDriveInputs {
        double currentRotationRadians;
        Pose2d pose;
        SwerveModuleState[] moduleStates;
        SwerveModuleState[] driveDesiredStates;
        SwerveModulePosition[] modulePositions;
        SwerveModuleInputsAutoLogged swerveModuleFL;
        SwerveModuleInputsAutoLogged swerveModuleFR;
        SwerveModuleInputsAutoLogged swerveModuleBR;
        SwerveModuleInputsAutoLogged swerveModuleBL;
        ChassisSpeeds robotRelativeSpeeds;
    }


    public default Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints) {
        return generateTrajectory(start, end, internalPoints, 0,0);
    }


    public default Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond) {
        TrajectoryConfig config = new TrajectoryConfig(0.1, 0.1);
        config.setEndVelocity(endVelocityMetersPerSecond);
        config.setStartVelocity(startVelocityMetersPerSecond);

        var trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                internalPoints,
                end,
                config
        );
        SmartDashboard.putNumber("Time", trajectory.getTotalTimeSeconds());
        SmartDashboard.putNumber("trajEndRotation", trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("desiredEndRot", end.getRotation().getDegrees());
        Field2d field = new Field2d();
        SwerveControllerCommand trajCommand = new SwerveControllerCommand(
                trajectory,
//                vision::getRobotPosition,
//             this::getEstimatedPose,
                this::getPose,
                kinematics,
                xController,
                yController,
                thetaController,
                this::setModuleStates,
                this
        );
//        field.getRobotObject().setTrajectory(trajectory);
//        SmartDashboard.putData("trajectory",field);
        Logger.recordOutput(("trajectory"), trajectory);
        return trajCommand;
    }
}
