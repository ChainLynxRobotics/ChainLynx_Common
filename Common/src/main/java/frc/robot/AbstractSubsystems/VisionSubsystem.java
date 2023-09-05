package frc.robot.AbstractSubsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Abstract class for auto-alignment to field markers such as April Tags; extend this parent class and override methods you want to customize to implement */
public abstract class VisionSubsystem extends SubsystemBase {

    /** if target(s) is present, update field-relative pose of robot */
    public abstract void updateOdometry(SwerveDrivePoseEstimator estimator);

    public abstract boolean getHasTarget();
}
