package frc.robot.AbstractSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Abstract class for single flywheel hooded shooter with binary state hood (pneumatics); extend this parent class and override methods you want to customize to implement */
public abstract class ShooterSubsystem extends SubsystemBase {
    
    /** @param goalAtTop set to true if shooter is actuated by a motor, and false if by pneumatics */
    public ShooterSubsystem(boolean goalAtTop) {}

    /** get initial launch conditions based off shooter zone relative to scoring location; shooter zone should be a global enum defined in Constants.java */
    public abstract<T> void setRPM(T shooterZone);

    /** set reference of PID controller and/or motion profile */
    public abstract void updateRPM(double setpointVelocity);

    /** set hood angle based off current location relative to scoring location */
    public abstract<T> void setAngle(T shooterZone, double setpoint);

    /** if motors are actuating hood, apply a control mechanism (e.g., PID) to calculate motor effort */
    public abstract void updateAngle(double setpoint);

    public abstract boolean isAtSetpoint();

    /** convert encoder rotations to angle based off the number of ticks per rotation of gear (set in Constants.java) */
    public abstract double tickToAngle(double ticks);

    /** based on flywheel radius set in Constants.java */
    public abstract double velocityToRPM(double velocity);

    public abstract<M> M getHoodMotor();

    public abstract<M> M getFlywheelMotor();

}
