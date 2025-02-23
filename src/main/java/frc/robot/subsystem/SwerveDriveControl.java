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
        return this.getSensors().pose;
    }
    default Pose2d getFieldPose(){
        return getPose();
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
        Pose2d Pose2dFromLL = new Pose2d();
        Pose2d NewPose2dFromLL = new Pose2d();
        ChassisSpeeds robotRelativeSpeeds;
    }


    public default Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints) {
        return generateTrajectory(start, end, internalPoints, 0,0);
    }


    public default Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond) {
        return Commands.none();
    }
}
