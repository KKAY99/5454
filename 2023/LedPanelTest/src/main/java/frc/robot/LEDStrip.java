package frc.robot;

import java.awt.Color;

import edu.wpi.first.hal.simulation.AddressableLEDDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
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
    private int m_animateStartPos=0;
    private int m_animateCurPos=0;
    private int m_animateEndPos=0;
    private int m_animateLoop=0;
    private double m_animateDelay=0;
    private double m_animateLastTime=0;
    public static final int MODE_WAVE = 0;
    public static final int MODE_BAR = 1;
    public static final int MODE_RAINBOW = 2;
    public static final int MODE_SOLID = 3;
    public static final int MODE_OFF = 4;
    private static final int LEDLETTERSIZE=32;
    private static final int [] letterA={8,9,10,11,12,13,14,15,16,18,19,24,25,26,27,28,29,30,31};
    private static final int [] letterB={8,9,10,11,12,13,14,15,16,19,20,23,24,25,26,29,30,31};
    private static final int [] letterC={8,9,10,11,12,13,14,15,16,23,24,31};
    private static final int [] letterD={8,9,10,11,12,13,14,15,16,23,25,26,27,28,29,30};
    private static final int [] letterE={8,9,10,11,12,13,14,15,16,17,19,20,22,23,24,25,27,28,30,31};
    private static final int [] letterF={8,9,10,11,12,13,14,15,16,17,19,20,27,28,30,31};
    private static final int [] letterG={8,9,10,11,12,13,14,15,16,20,23,24,25,26,27,30,31};
    private static final int [] letterH={8,9,10,11,12,13,14,15,19,20,24,25,26,27,28,29,30,31};
    private static final int [] letterI={8,9,14,15,16,17,18,19,20,21,22,23,24,25,30,31};
    private static final int [] letterJ={8,9,10,11,22,23,24,25,26,27,28,29,30,31};
    private static final int [] letterK={8,9,10,11,12,13,14,15,19,20,24,25,26,29,30};
    private static final int [] letterL={8,9,10,11,12,13,14,15,22,23,24,25};
    private static final int [] letterM={8,9,10,11,12,13,14,15,17,18,19,24,25,26,27,28,29,30,31};
    private static final int [] letterN={8,9,10,11,12,13,14,15,16,17,24,25,26,27,28,29,30,31};
    private static final int [] letterO = {8,9,10,11,12,13,14,15,16,23,24,25,26,27,28,29,30,31};
    private static final int [] letterP={8,9,10,11,12,13,14,15,16,19,28,29,30,31};
    private static final int [] letterU={8,9,10,11,12,13,14,15,22,23,24,25,26,27,28,29,30,31};
    private static final int [] letterR={8,9,10,11,12,13,14,15,16,18,19,24,25,26,27,30,31};
    private static final int [] letterS={8,12,13,14,15,16,19,20,23,24,25,26,27,31};
    private static final int [] letterT={14,15,16,17,18,19,20,21,22,23,30,31};
    private static final int [] shapeCube = {8,9,10,11,12,13,14,15,16,23,24,31,32,39,40,47,48,49,50,51,52,53,54,55};
    private static final int [] shapeCone = {8,9,20,21,23,24,28,29,32,33,39,40,44,45,52,53,55,56,57};
   
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
        System.out.println("mode - " + this.mode);
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
    public void animationSeup(int startPos,int endPos,int loopCount,double delayTime){
        m_animateStartPos=startPos;
        m_animateCurPos=startPos;
        m_animateEndPos=endPos;
        m_animateLoop=loopCount;
        m_animateDelay=delayTime;
    }
    public void animatethis(String animateCommand,int hue){
        double currentTime = Timer.getFPGATimestamp();
        // only update if delay time has occured
        if(currentTime-m_animateDelay>m_animateLastTime){
            int ledPos=m_animateCurPos-m_animateLoop;
            if(ledPos<m_animateEndPos){
                ledPos=m_animateStartPos;
            }
            switch(animateCommand){
                case "5454":
                    show5454(ledPos,hue);
                    break;
                case "cone":
                    showCone(ledPos,hue);
                    break;
                case "cube":
                    showCube(ledPos, hue);
                    break;
            }
     
            m_animateCurPos=ledPos;   
            m_animateLastTime=currentTime;
        }    
    }

    public void animate5454(){
        double currentTime = Timer.getFPGATimestamp();
        // only update if delay time has occured
        if(currentTime-m_animateDelay>m_animateLastTime){
            int ledPos=m_animateCurPos-m_animateLoop;
            if(ledPos<m_animateEndPos){
                ledPos=m_animateStartPos;
            }
            show5454(ledPos,150);
            m_animateCurPos=ledPos;   
            m_animateLastTime=currentTime;
        }    
    }
    private void clearStrip(){
        int ledLoop=0;
        while (ledLoop<(m_ledBuffer.getLength())){
            m_ledBuffer.setHSV(ledLoop,0,0,0);
            ledLoop++;
        }
    }
    public void clearLED(){
        clearStrip();
        m_led.setData(m_ledBuffer);
 
    }

    private void showArray(int startingLED,int hue, int[] ledMap){
        int ledCount=ledMap.length;
        int iLedLoop=0;
        int ledPos=0;
        while(iLedLoop<ledCount){
            ledPos=ledMap[iLedLoop]+startingLED;
            if(ledPos<Constants.LEDS.COUNT){
              m_ledBuffer.setHSV(ledPos,hue,255,255);
            }
              iLedLoop++;
        }

    }
    public void showCone(int startingLED,int hue){
        clearStrip();
       
        int iCurrentPos=startingLED;
        showArray(iCurrentPos,hue, letterC);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterO);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterN);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterE);
        
        iCurrentPos=iCurrentPos+LEDLETTERSIZE+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, shapeCone);
      
        //Update Display
        m_led.setData(m_ledBuffer);
 
    }
    public void showObsidian(int startingLED){
        clearStrip();
        int hue=150;
        int iCurrentPos=startingLED;
        showArray(iCurrentPos,hue, letterO);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterB);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterS);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterI);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterD);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterI);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterA);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterN);
        
        
 
     //Update Display
        m_led.setData(m_ledBuffer);
 
    }
   
    public void showCube(int startingLED,int hue){
        clearStrip();
       
        int iCurrentPos=startingLED;
        showArray(iCurrentPos,hue, letterC);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterU);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterB);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterE);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, shapeCube);
      
 
     //Update Display
        m_led.setData(m_ledBuffer);
 
    }
    public void showGoBlue(int startingLED){
        clearStrip();
        int hue=120;
        int iCurrentPos=startingLED;
        showArray(iCurrentPos,hue, letterG);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterO);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterB);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterL);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterU);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        showArray(iCurrentPos,hue, letterE);
        iCurrentPos=iCurrentPos+LEDLETTERSIZE;
        //Update Display
        m_led.setData(m_ledBuffer);
 
    }
    public void show5454(int startingLED,int hue){
        clearStrip();
        
        m_ledBuffer.setHSV(startingLED+12, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+13, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+14, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+15, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+8, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+16, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+19, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+23, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+31, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+27, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+26, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+25, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+47, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+46, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+45, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+44, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+51, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+63, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+62, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+61, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+60, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+59, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+58, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+57, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+56, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+79, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+78, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+77, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+76, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+72, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+80, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+83, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+95, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+80, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+91, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+90, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+89, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+87, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+72, hue, 255, 255);
        m_ledBuffer.setHSV( startingLED+111, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+110, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+109, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+108, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+115, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+127, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+126, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+125, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+124, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+123, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+122, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+121, hue, 255, 255);
        m_ledBuffer.setHSV(startingLED+120, hue, 255, 255);
        m_led.setData(m_ledBuffer);
 
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