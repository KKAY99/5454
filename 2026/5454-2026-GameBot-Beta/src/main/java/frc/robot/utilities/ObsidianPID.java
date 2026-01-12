package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;

public class ObsidianPID{
    private double m_p;
    private double m_i;
    private double m_d;
    private double m_max;
    private double m_min;
    
    private double m_inputGain=1;

    private double m_iAccum;
    private double m_lastError;
    private double m_prevTimeStamp=0;

    private boolean m_toggle=false;

    public ObsidianPID(double p,double i,double d,double max,double min){
        m_p=p;
        m_i=i;
        m_d=d;
        m_max=max;
        m_min=min;

        m_iAccum=0;
        m_lastError=0;
    }

    public void setInputGain(double inputGain){
        m_inputGain=inputGain;
    }

    public void setAllValues(double p,double i,double d,double max,double min,double inputGain){
        m_p=p;
        m_i=i;
        m_d=d;
        m_max=max;
        m_min=min;
        m_inputGain=inputGain;
    }

    public void togglePID(){
        m_toggle=m_toggle?false:true;
        m_prevTimeStamp=Timer.getFPGATimestamp();
        m_iAccum=0;
        m_lastError=0;
    }

    public boolean getToggle(){
        return m_toggle;
    }

    public void resetToggle(){
        m_toggle=false;
    }

    public double calculatePercentOutput(double pos,double setPoint){
        double currentTimeStamp=Timer.getFPGATimestamp();
        pos=(m_inputGain!=1)?pos*m_inputGain:pos;
        setPoint=(m_inputGain!=1)?setPoint*m_inputGain:setPoint;

        double error=setPoint-pos;
        double errorRate=(m_lastError!=0)?error-m_lastError/currentTimeStamp-m_prevTimeStamp:0;
        m_iAccum=m_iAccum+error*(currentTimeStamp-m_prevTimeStamp);

        double calculatedOutput=(m_p*error)+(m_iAccum*m_i)+(m_d*errorRate);
        calculatedOutput=(calculatedOutput>m_max)?m_max:calculatedOutput;
        calculatedOutput=(calculatedOutput<m_min)?m_min:calculatedOutput;

        return calculatedOutput*-1;
    }
    
}
