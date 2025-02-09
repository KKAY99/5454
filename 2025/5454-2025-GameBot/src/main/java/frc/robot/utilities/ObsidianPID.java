package frc.robot.utilities;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ObsidianPID{
    private double m_p;
    private double m_i;
    private double m_d;
    private double m_max;
    private double m_min;

    private double m_currentError;
    private double m_iAccum;
    private double m_lastError;

    public ObsidianPID(double p,double i,double d,double max,double min){
        m_p=p;
        m_i=i;
        m_d=d;
        m_max=max;
        m_min=min;

        m_iAccum=0;
        m_lastError=0;
    }

    public double calculatePercentOutput(DoubleSupplier pos,DoubleSupplier setPoint){
        double error=setPoint.getAsDouble()-pos.getAsDouble();
        double errorRate=(m_lastError!=0)?error-m_lastError:0;
        m_iAccum=m_iAccum+m_i*error;

        m_currentError=error;
        return (m_p*error)+(m_iAccum)+(m_d*errorRate);
    }
    
}
