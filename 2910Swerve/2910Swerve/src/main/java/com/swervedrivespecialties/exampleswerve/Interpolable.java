package com.swervedrivespecialties.exampleswerve;


public interface Interpolable<T> {
    T interpolate(T other, double t);
}
