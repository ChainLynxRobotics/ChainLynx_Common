package frc.robot.Utils.Interpolators;

/*returns key from value and interpolation curve */
public interface InverseInterpolator<T> {
    
    public double inverseInterpolate(T interpolationType, T key);
}
