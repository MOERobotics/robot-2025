package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    public SwerveModule SwervemoduleFL;
    public SwerveModule SwervemoduleFR;
    public SwerveModule SwervemoduleBL;
    public SwerveModule SwervemoduleBR;
    public SwerveDriveKinematics kinematics;

    public static class Inputs {
        double initialRotation;
        double currentRotation;
    }
public SwerveDrive(
        SwerveModule SwerveModuleFL,
        SwerveModule SwerveModuleFR,
        SwerveModule SwerveModuleBR,
        SwerveModule SwerveModuleBL){
        this.SwervemoduleBR = SwerveModuleBR;
    this.SwervemoduleBL = SwerveModuleBL;
    this.SwervemoduleFR = SwerveModuleFR;
    this.SwervemoduleFL = SwerveModuleFL;
    kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveModuleFL.xpos, SwerveModuleFL.ypos ),
            new Translation2d(SwerveModuleFR.xpos, SwerveModuleFR.ypos ),
            new Translation2d(SwerveModuleBR.xpos, SwerveModuleBR.ypos ),
            new Translation2d(SwerveModuleBL.xpos, SwerveModuleBL.ypos )
    );
}
public void Drive(double xpower, double ypower, double turnspeed

) {
   ChassisSpeeds Cspeeds = new ChassisSpeeds(xpower, ypower, turnspeed);
   SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(Cspeeds);
   SwervemoduleFL.drive(moduleStates[0].speedMetersPerSecond);
   SwervemoduleFL.pivot(moduleStates[0].angle.getMeasure());
    SwervemoduleFL.drive(moduleStates[1].speedMetersPerSecond);
    SwervemoduleFR.pivot(moduleStates[1].angle.getMeasure());
    SwervemoduleBR.drive(moduleStates[2].speedMetersPerSecond);
    SwervemoduleBR.pivot(moduleStates[2].angle.getMeasure());
    SwervemoduleBL.drive(moduleStates[3].speedMetersPerSecond);
    SwervemoduleBL.pivot(moduleStates[3].angle.getMeasure());


}
}
