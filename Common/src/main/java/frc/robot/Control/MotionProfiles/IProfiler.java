package frc.robot.Control.MotionProfiles;

public interface IProfiler {

    public interface Config<T extends Config<T>> {

        public boolean atConfig(T config);
    }

    public double calculate(double t);

    public boolean setpointReached(double curPos, double setpoint, double error);

}
