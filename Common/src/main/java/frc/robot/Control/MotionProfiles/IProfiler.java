package frc.robot.Control.MotionProfiles;

public interface IProfiler {

    public interface Config {

        public boolean atConfig();
    }

    public double calculate(double t);

    public boolean setpointReached(double curPos, double setpoint, double error);

}