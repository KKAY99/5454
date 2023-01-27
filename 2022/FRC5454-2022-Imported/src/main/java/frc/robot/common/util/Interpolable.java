package frc.robot.common.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
