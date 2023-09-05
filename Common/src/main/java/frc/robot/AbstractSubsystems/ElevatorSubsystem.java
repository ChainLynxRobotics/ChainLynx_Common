package frc.robot.AbstractSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Abstract class for single or multi-stage elevator; extend this parent class and override methods you want to customize to implement */
public abstract class ElevatorSubsystem extends SubsystemBase {
    
    /** set the reference for the PID/motion profile controlling elevator motors */
    public abstract void moveElevator();

    /** if using custom PID, calculate and set the control effort here */
    public abstract void applyControlEffort(double current, double setpoint);

    /** set leader motor speed */
    public abstract void setMotors(double speed);

    public abstract void zeroEncoders();

    public abstract void setElevatorSetpoint(double setpoint);

    public abstract void getElevatorSetpoint();
}
