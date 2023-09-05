package frc.robot.AbstractSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Abstract class for intaking game pieces; extend this parent class and override methods you want to customize to implement */
public abstract class IntakeSubsystem extends SubsystemBase {
    
    public abstract void intakeGamePiece(double speedMultiplier);

    public abstract void releaseGamePiece(double speedMultiplier);

    public abstract void stopMotors();

    /** get from a joystick axis or shuffleboard setting */
    public abstract double getSpeedMultiplier();

    /*T should be an global enum for the types of gamepieces in the season's game, to be defined in Constants.java */
    public abstract<T> T getGamePieceType(T gamePiece);
}
