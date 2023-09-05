package frc.robot.AbstractSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Abstract class for coaxial swerve drive; extend this parent class and override methods you want to customize to implement */
public abstract class DrivetrainSubsystem extends SubsystemBase {
    protected PowerDistribution pDist; 

    public DrivetrainSubsystem() {
        pDist.clearStickyFaults();
    }

    public abstract Pose2d getPose();

    /** zero swerve modules, zero gyro, reset encoders, etc. */
    public abstract void resetOdometry(Pose2d pose);

    /** should be called periodically */
    public abstract void drive(double xSpeed, double ySpeed, double xRot, double yRot);

    /** specify if driving field-relative or robot-relative, and if gyro needs to be realigned; should be called periodically*/
    public abstract void drive(double xSpeed, double ySpeed, double xRot, double yRot, boolean altDrive, boolean centerGyro);

    /** set swerve modules into x position to prevent movement */
    public abstract void setX();

    public abstract void setSwerveModuleStates();

    public abstract SwerveModuleState[] getSwerveModuleStates();

    public abstract double getHeading();

    /** get speed at which robot is turning, preferably using gyro */
    public abstract double getTurnRate();
}

