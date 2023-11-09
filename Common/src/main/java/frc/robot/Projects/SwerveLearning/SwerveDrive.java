package frc.robot.SwerveLearning;

import com.ctre.phoenix.sensors.WPI_Pigeon2; //gyroscope (detects rotation)
import edu.wpi.first.math.geometry.Pose2d; //pose2d is a rotation2d and a translation2d
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds; //does some vector math for you to convert from robot's field-relative speed to speed of the robot chassis in the robot's frame of reference
import edu.wpi.first.math.kinematics.SwerveDriveKinematics; //performs inverse or forward kinematics to convert from individual swerve module speeds to chassis speed and vice versa
import edu.wpi.first.math.kinematics.SwerveDriveOdometry; //get robot's position on the field using swerve module encoders and angle using robot gyroscope
import edu.wpi.first.math.kinematics.SwerveModulePosition; //store positions of swerve modules
import edu.wpi.first.math.kinematics.SwerveModuleState; //speed and rotation2d angle

//look at documentation here for anything WPILib: https://first.wpi.edu/wpilib/allwpilib/docs/release/java/index.html

public class SwerveDrive extends DrivetrainSubsystem {

    /* 1: Construct 4 swerve module objects
     * in Constants file, create a static class to store turning and translating motor ids (for now pick small distinct integers) and angular offsets
     * angular offsets are just each module's initial rotation in radians (e.g,. let one module have a 0 rad offset and the other all 1/4th of a full rotation away from the adjacent modules)
    */
    

    /* 2: Construct a gyroscope object; assign it a different id than the swerve module motor controllers */

    /* 3: create swerve drive odometry object
     * you can copy the SwerveDriveKinematics object from the Constants.java class of our 2023 code*/ 

    /* 4: override periodic method and update odometry object inside 
     *SwerveDrive indirectly extends SubsystemBase; Swerve Drive extends DrivetrainSubsystem which extends SubsystemBase
     *within update method call, pass in gyro angle and a swerve module position array
    */

    @Override
    public Pose2d getPose() {
        //5: get robot pose from odometry project
        throw new UnsupportedOperationException("Unimplemented method 'getPose'"); //placeholder; remove this once done with your implementation
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        //6: reset position registered by odometry object
        throw new UnsupportedOperationException("Unimplemented method 'resetOdometry'");
    }


    @Override
    public void drive(double xSpeed, double ySpeed, double xRot, double yRot, boolean altDrive, boolean centerGyro) {
        /*12: reset gyro heading (angle reading to 0) if center gyro true*/

        /*13: if altDrive is true, we are moving relative to the robot, so whatever the direction the robot is facing is forward
         * if altDrive is false, we are moving relative to the field, so the orientation of the robot doesn't determine the direction it is moving; it has a fixed reference frame
         * 
         * to implement this, define a variable to track rotation
         * if altDrive is false, set rotation to be xRot times the drivetrain's maximum angular speed (add this to your constants file from the 2023 season code)
         * if altDrive is true, calculate the angle between the robot's orientation and input joystick values (xRot and yRot)
         *            you will need trig for this (ask if you need help starting)
         *            add the gyroscope angle to the stick angle (always keep answer <360 degrees)
         *            use hyperbolic tangent (tanh) from the Math library to scale you value to be between -1 and 1
        */


        /*14: scale x,y, and angular velocities by maxima and get swerve module states from drivetrain kinematics object
         * renormalize wheel speeds
         * set desired state of swerve modules from swerve module state variable
         */
    }


    @Override
    public void setX() {
        //7: set the state of swerve modules in an x position to prevent movement
    }

    @Override
    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        /*8: if any desired wheel speed is > maxSpeed, renormalize speeds of modules; check out docs at edu.wpi.first.math.kinematics
        set corresponding state for each swerve module object*/
    }

    @Override
    public SwerveModuleState[] getSwerveModuleStates() {
        /*9: return an array of swerve module states for each swerve module object*/
        throw new UnsupportedOperationException("Unimplemented method 'getSwerveModuleStates'");
    }

    @Override
    public double getHeading() {
        /*10: insert gyro object*/
        //return Rotation2d.fromDegrees(-[gyro object here].getAngle()).getDegrees();
        throw new UnsupportedOperationException("Unimplemented method 'getHeading'");
    }

    @Override
    public double getTurnRate() {
        /*11: get turn rate of gyro object*/
        throw new UnsupportedOperationException("Unimplemented method 'getTurnRate'");
    }
    
}
