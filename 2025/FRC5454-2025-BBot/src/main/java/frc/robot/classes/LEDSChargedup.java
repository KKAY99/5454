package frc.robot.classes;

import frc.robot.Constants;
import frc.robot.Constants.LEDS.Colors;

public class LEDSChargedup {
    private static final int LEDMODE_WAVE = 0;
    private static final int LEDMODE_BAR = 1;
    private static final int LEDMODE_RAINBOW = 2;
    private static final int LEDMODE_SOLID = 3;
    private static final int LEDMODE_OFF = 4;
    private static boolean m_ledFlash = false;
    private static boolean m_ledFlashState = false;

    public static enum LEDMode {
        NOTSET, DISABLED, AUTOMODE, TELEOP, BALANCED, CANSEETARGET, CANTSEETARGET, AUTOSCORING, RETRACTING;
    }

    private LEDStrip m_upperLEDStrip;
    // private LEDStrip m_bottomLEDStrip;
    private LEDMode m_oldLEDMode;
    private LEDMode m_LEDMode;
    private int m_ledFlashDelayCount = 0;
    public int m_pipeline;
    public boolean m_canSeeTarget;

    public LEDSChargedup(int upperPort, int ledCountUpper) {
        try {
            m_upperLEDStrip = new LEDStrip(upperPort, ledCountUpper);
            // m_bottomLEDStrip=new LEDStrip(bottomPort,ledCountBottom);
            m_upperLEDStrip.setMode(LEDMODE_WAVE);
            m_upperLEDStrip.setColor(Colors.PURPLE);
            // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
            // m_bottomLEDStrip.setColor(Colors.PURPLE);

            m_oldLEDMode = LEDMode.NOTSET;
        } catch (Exception e) {
            System.out.println("LED Failure");
            System.out.println("Exception Message: " + e.getMessage());
            System.out.println("StackTrace:" + e.getStackTrace().toString());
        }

    }

    public LEDSChargedup() {}

    public void updateLED() {
        try {

            if (m_oldLEDMode != m_LEDMode) {
                m_oldLEDMode = m_LEDMode;
                // System.out.println("update LED");
            } else {
                // TODO: Fix Flash LED Lights
                if (m_ledFlash) {
                    flashmode();
                }
                m_upperLEDStrip.update();
                // m_bottomLEDStrip.update();

            }

        } catch (Exception e) {
            // System.out.println("LED Failure");
            // System.out.println("Exception Message: " + e.getMessage());
            // System.out.println("StackTrace:" + e.getStackTrace().toString());
        }

    }

    public void setRobotMode(LEDMode ledMode) {
        try {
            m_LEDMode = ledMode;
            switch (ledMode) {
                case NOTSET:
                    break;
                case DISABLED:
                    m_upperLEDStrip.setMode(LEDMODE_WAVE);
                    m_upperLEDStrip.setColor(Colors.PURPLE);
                    // m_bottomLEDStrip.setMode(LEDMODE_WAVE);
                    // m_bottomLEDStrip.setColor(Colors.PURPLE);
                    m_ledFlash = false;
                    break;
                case AUTOMODE:
                    m_upperLEDStrip.setColor(Colors.PURPLE);
                    m_upperLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setColor(Colors.PURPLE);
                    // m_bottomLEDStrip.setMode(LEDMODE_RAINBOW);
                    m_ledFlash = false;
                    break;
                case TELEOP:
                    m_upperLEDStrip.setMode(LEDMODE_WAVE);
                    m_upperLEDStrip.setColor(Colors.GREEN);
                    // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setColor(Colors.GREEN);

                    m_ledFlash = false;
                    if (m_canSeeTarget == false) {
                        // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                        // m_bottomLEDStrip.setColor(Colors.RED);
                    }
                    break;
                case BALANCED:
                    // m_bottomLEDStrip.setMode(LEDMODE_WAVE);
                    // m_bottomLEDStrip.setColor(Colors.PURPLE);
                    m_ledFlash = true;

                    break;
                case RETRACTING:
                    // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setColor(Colors.BLUE);
                    m_ledFlash = false;

                    break;
                case CANSEETARGET:
                    // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setColor(Colors.GREEN);
                    m_ledFlash = true;

                    break;
                case CANTSEETARGET:
                    // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setColor(Colors.RED);
                    m_ledFlash = false;

                    break;
                case AUTOSCORING:
                    m_upperLEDStrip.setMode(LEDMODE_SOLID);
                    m_upperLEDStrip.setColor(Colors.GREEN);
                    // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setColor(Colors.GREEN);
                    m_ledFlash = true;
            }

            updateLED();
        } catch (Exception e) {
            // System.out.println("LED Failure");
            // System.out.println("Exception Message: " + e.getMessage());
            // System.out.println("StackTrace:" + e.getStackTrace().toString());
        }

    }

    public void flashmode() {
        try {
            m_ledFlashDelayCount++;
            if (m_ledFlashState == false) {
                // (m_ledFlashDelayCouunt>10)
                if (m_ledFlashDelayCount > Constants.LEDS.FLASH_DELAY) {
                    m_upperLEDStrip.setMode(LEDMODE_OFF);
                    // m_bottomLEDStrip.setMode(LEDMODE_OFF);
                    m_ledFlashState = true;
                    m_ledFlashDelayCount = 0;
                }
            } else {
                if (m_ledFlashDelayCount > Constants.LEDS.FLASH_DELAY) {
                    m_upperLEDStrip.setMode(LEDMODE_SOLID);
                    // m_bottomLEDStrip.setMode(LEDMODE_SOLID);
                    m_ledFlashState = false;
                    m_ledFlashDelayCount = 0;

                }
            }

        } catch (Exception e) {
            // System.out.println("LED Failure");
            // System.out.println("Exception Message: " + e.getMessage());
            // System.out.println("StackTrace:" + e.getStackTrace().toString());
        }

    }

    public void setPipelineLED(int led){
        //Bad leds
    }

    public void XsetPipelineLED(int overrideValue) {
        try {
            if (m_LEDMode == LEDMode.AUTOSCORING || m_LEDMode == LEDMode.DISABLED || m_LEDMode == LEDMode.AUTOMODE) {
            } else {
                switch (m_pipeline) {
                    case Constants.VisionPipelines.AprilTag:
                        m_upperLEDStrip.setMode(LEDMODE_SOLID);
                        m_upperLEDStrip.setColor(Colors.PURPLE);
                        m_ledFlash = true;
                        break;
                    case Constants.VisionPipelines.TopTape:
                    case Constants.VisionPipelines.BottomTape:
                        m_upperLEDStrip.setMode(LEDMODE_SOLID);
                        m_upperLEDStrip.setColor(Colors.YELLOW);
                        m_ledFlash = true;
                        break;
                }
            }
            updateLED();
        } catch (Exception e) {
            // System.out.println("LED Failure");
            // System.out.println("Exception Message: " + e.getMessage());
            // System.out.println("StackTrace:" + e.getStackTrace().toString());
        }

    }

    public void setPipelineLED() {
        setPipelineLED(m_pipeline);
    }
}
