package frc.robot.AbstractSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Abstract class for pneumatically actuated arm; extend this parent class and override methods you want to customize to implement*/
public abstract class ArmSubsystem extends SubsystemBase {
    
    public abstract void expand();

    public abstract void retract();

    public abstract double getPressure();
}


