package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveControl extends Subsystem {
    @AutoLog
    class SwerveDriveInputs {
        public double currentRotationRadians;
        public Pose2d pose;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] driveDesiredStates;
        public SwerveModulePosition[] modulePositions;
        public SwerveModuleInputsAutoLogged swerveModuleFL;
        public SwerveModuleInputsAutoLogged swerveModuleFR;
        public SwerveModuleInputsAutoLogged swerveModuleBR;
        public SwerveModuleInputsAutoLogged swerveModuleBL;
        public ChassisSpeeds robotRelativeSpeeds;
    }

    default Pose2d getPose() {
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

    enum CommandType {
        QuasistaticForward, QuasistaticReverse, DynamicForward, DynamicReverse
    }

    enum ModuleType {
        modFL, modFR, modBL, modBR, allMods
    }

    enum DriveOrPivot {
        setDrive, setPivot,
    }

    default public Command sysIDCommandsPivot(CommandType commandType, ModuleType moduleType) {
        return Commands.none();
    }

    default public Command sysIDCommandsDrive(CommandType commandType) {
        return Commands.none();
    }

    void driveSingleModule(int index, double xSpeed, double ySpeed, double rotation);

    default ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getSensors().robotRelativeSpeeds;
    }

    SwerveDriveKinematics getKinematics();


    SwerveDriveOdometry getOdometry();

    void pivotAngle(Angle angle);
}
