package frc.robot.subsystem;

import com.fasterxml.jackson.databind.introspect.Annotated;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

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
}
