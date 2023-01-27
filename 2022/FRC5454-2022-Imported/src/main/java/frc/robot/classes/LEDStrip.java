package frc.robot.classes;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.Constants.LEDS.Colors;

public class LEDStrip {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;
    private int iteration = 0;
    private boolean entering = true;
    private long lastTime = System.currentTimeMillis();
    private int color = 0;
    private int mode = 0;
    private int percentage = 0;
    private int start = 0;

    public static final int MODE_WAVE = 0;
    public static final int MODE_BAR = 1;
    public static final int MODE_RAINBOW = 2;
    public static final int MODE_SOLID = 3;
    public static final int MODE_OFF = 4;

    public LEDStrip(int port, int leds) {
        // initialize all relevant objects
        m_led = new AddressableLED(port);

        m_ledBuffer = new AddressableLEDBuffer(leds);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public LEDStrip(int port, int leds, int start) {
        // initialize all relevant objects
        m_led = new AddressableLED(port);
        this.start = start;

        m_ledBuffer = new AddressableLEDBuffer(leds + this.start);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void update() {
       // System.out.println("mode - " + this.mode);
        if (this.mode == 0 || this.mode == 3) {
            this.wave();
        } else if (this.mode == 1) {
            this.bar();
        } else if (this.mode == 2) {
            this.rainbow();
            m_led.setData(m_ledBuffer);
        } else if (this.mode == 4) {
            for (int i = 0; i < this.m_ledBuffer.getLength() - start; i++) {
                m_ledBuffer.setHSV(i + start, 0, 255, 0);
            }
            m_led.setData(m_ledBuffer);
        }
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength() - start; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i + start, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    private void wave() {
        // set the hue to the selected color
        
        int hue = 0;
        switch (color) {
            case Colors.RED:
                hue = 0;
                break;

            case Colors.PINK:
                hue = 175;
                break;

            case Colors.PURPLE:
                hue = 150;
                break;

            case Colors.BLUE:
                hue = 120;
                break;

            case Colors.CYAN:
                hue = 100;
                break;

            case Colors.GREEN:
                hue = 45;
                break;

            case Colors.YELLOW:
                hue = 20;
                break
                
                ;

            case Colors.ORANGE:
                hue = 11;
                break;
        }
        //System.out.println("LED H" + hue + " - " + color );
        if (this.mode == 0) {
            // draw leds as animation
            if (this.entering) {
                for (int i = 0; i < this.iteration; i++) {
                    m_ledBuffer.setHSV(i + start, hue, 255, 255);
                }

                for (int i = this.iteration; i < this.m_ledBuffer.getLength() - iteration - start; i++) {
                    m_ledBuffer.setHSV(i + start, hue, 255, 0);
                }

                for (int i = this.m_ledBuffer.getLength() - iteration - start; i < this.m_ledBuffer.getLength()
                        - start; i++) {
                    m_ledBuffer.setHSV(i + start, hue, 255, 255);
                }
            } else {
                for (int i = 0; i < this.iteration; i++) {
                    m_ledBuffer.setHSV(i + start, hue, 255, 0);
                }

                for (int i = this.iteration; i < this.m_ledBuffer.getLength() - iteration - start; i++) {
                    m_ledBuffer.setHSV(i + start, hue, 255, 255);
                }

                for (int i = this.m_ledBuffer.getLength() - iteration - start; i < this.m_ledBuffer.getLength()
                        - start; i++) {
                    m_ledBuffer.setHSV(i + start, hue, 255, 0);
                }
            }

            if (System.currentTimeMillis() - 25 > this.lastTime) {
                this.iteration++;
                this.lastTime = System.currentTimeMillis();
            }
            if (this.iteration == (this.m_ledBuffer.getLength() / 2) + 1) {
                this.iteration = 0;
                this.entering = !this.entering;
            }
        } else {
            for (int i = 0; i < this.m_ledBuffer.getLength() - start; i++) {
                this.m_ledBuffer.setHSV(i + start, hue, 255, 255);
            }
        }

        m_led.setData(m_ledBuffer);
    }

    private void bar() {
        // set the hue to the selected color
        int hue = 0;
        if (this.percentage > 33) {
            hue = 11;
        }

        if (this.percentage > 66) {
            hue = 20;
        }

        if (this.percentage >= 99) {
            hue = 45;
        }

        int half = this.m_ledBuffer.getLength() / 2;
        double percentage = this.percentage / 100.0;
        if (percentage > 1.0) {
            percentage = 1.0;
        }

        double size = percentage * half;
        long round = Math.round(size);
        for (int i = 0; i < this.m_ledBuffer.getLength() - start; i++) {
            m_ledBuffer.setHSV(i + start, 0, 0, 0);
        }

        for (long i = half - round; i < half + round; i++) {
            m_ledBuffer.setHSV((int) i + start, hue, 255, 255);
        }

        m_led.setData(m_ledBuffer);
    }

    public void setColor(int newcolor) {
        this.color = newcolor;
    }
    public int getColor(){
        return this.color;
    }

    public void setPercentage(int percentage) {
        this.percentage = percentage;
    }

    public void setMode(int newmode) {
        this.mode = newmode;
    }

    public int getMode() {
        return this.mode;
    }

}