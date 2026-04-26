package frc.robot.utilities;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * Subsystem for controlling CTRE CANdle LED strips with animations and color control.
 * 
 * <p>This subsystem provides a high-level interface for:
 * <ul>
 *   <li>Creating and configuring CANdle devices</li>
 *   <li>Defining LED strip segments</li>
 *   <li>Setting solid colors on strips</li>
 *   <li>Running various animations (countdown, breathing, etc.)</li>
 *   <li>Configuring animation parameters</li>
 * </ul>
 * 
 * <p>Example usage:
 * <pre>
 * CANdleLib ledLib = new CANdleLib(1, new LEDConfigs(), 100);
 * CANdle candle = ledLib.createCANdle();
 * LEDStrip strip = ledLib.createLEDStrip(0, 60);
 * 
 * // Set solid color
 * ledLib.setStripColor(candle, strip, Colors.RED).schedule();
 * 
 * // Run animation
 * Animations breathe = ledLib.createAnimation(candle, strip, Colors.BLUE, 0.5, 0.3, 0);
 * breathe.run();
 * </pre>
 */
public class CANdleLib{
    private int busId;
    private LEDConfigs m_rgbOrder;
    public int ledCount;
    private LEDConfigs globalBrightness;

    /**
     * Constructs a new CANdleLib with the specified CAN ID and LED strip type.
     * 
     * @param id the CAN bus ID of the CANdle device
     * @param rgbOrder the LED strip type defining the RGB byte order (e.g., RGB, GRB, BRG)
     * @param ledCount the total number of LEDs connected to the CANdle
     * @param globalBrightness the brightness of the leds
     */
    public CANdleLib(int id, double rgbOrder, int ledCount, double globalBrightness) {
        busId = id;
        this.m_rgbOrder = new LEDConfigs().withStripType(StripTypeValue.valueOf((int)rgbOrder));
        this.ledCount = ledCount;
        this.globalBrightness = new LEDConfigs().withBrightnessScalar(globalBrightness);
    }
    
    /**
     * Creates and configures a new CANdle device instance.
     * 
     * <p>This method initializes a CANdle with the CAN ID and LED strip type
     * specified in the constructor. The configuration is automatically applied
     * to the device.
     * 
     * @return a configured CANdle instance ready for use
     */
    public CANdle createCANdle() {
        CANdle candle;
        candle = new CANdle(busId,"5454Canivore");
        CANdleConfiguration config = new CANdleConfiguration();
        config.withLED(m_rgbOrder);
        config.withLED(globalBrightness);
        candle.getConfigurator().apply(config);
        
        return candle;
    }

    /**
     * Creates a logical LED strip segment definition.
     * 
     * <p>LED strips can be divided into multiple segments, each controllable
     * independently. This method creates a segment spanning from startIndex
     * (inclusive) to endIndex (exclusive).
     * 
     * @param startIndex the first LED index in the strip segment (inclusive)
     * @param endIndex the LED index after the last LED in the segment (exclusive)
     * @return an LEDStrip record representing the specified segment
     */
    public LEDStrip createLEDStrip(int startIndex, int endIndex) {
        if (startIndex < 0 || endIndex - startIndex <= 0) {
            throw new IllegalArgumentException("Check Parameters");
        }
        return new LEDStrip(startIndex, endIndex);
    }

    /**
     * Creates a state indicator animation that displays different colors based on an enum state.
     * 
     * <p>This animation maps enum ordinal values to colors from the provided color array.
     * When the state changes, the LED strip automatically updates to show the corresponding color.
     * If there are more states than colors, the mapping wraps around using modulo.
     * 
     * @param candle the CANdle device to control
     * @param strip the LED strip segment to animate
     * @param possibleStates a supplier that returns the current enum state
     * @param colors the array of colors to map to state ordinal values
     * @return an Animations instance for the state indicator
     */
    public Animations createAnimation(CANdle candle, LEDStrip strip, Supplier<Enum<?>> possibleStates, LEDColor... colors) {
        return new StateIndicator(candle, strip, possibleStates, colors); 
    }

    /**
     * Creates a range value animation that displays a numeric value as a progress bar.
     * 
     * <p>This animation visualizes a numeric value within a specified range by lighting
     * a proportional number of LEDs. LEDs representing the current value are shown in the
     * fill color, while remaining LEDs use the empty color. The value is automatically
     * clamped to the min/max range.
     * 
     * @param candle the CANdle device to control
     * @param strip the LED strip segment to animate
     * @param min the minimum value of the range
     * @param max the maximum value of the range
     * @param value a supplier that returns the current value to display
     * @param fillColor the color for LEDs representing the current value
     * @param emptyColor the color for LEDs beyond the current value
     * @return an Animations instance for the range value display
     */
    public Animations createAnimation(CANdle candle, LEDStrip strip, double min, double max, DoubleSupplier value, LEDColor fillColor, LEDColor emptyColor) {
        return new RangeValue(candle, strip, min, max, value, fillColor, emptyColor); 
    }

    /**
     * Creates a boolean indicator animation that displays one of two colors based on a boolean state.
     * 
     * <p>This animation switches between two colors depending on the boolean state returned by
     * the supplier. When the state is true, the strip shows trueColor; when false, it shows falseColor.
     * This is useful for indicating binary states like sensor readings or system status.
     * 
     * @param candle the CANdle device to control
     * @param strip the LED strip segment to animate
     * @param state a supplier that returns the current boolean state
     * @param trueColor the color to display when state is true
     * @param falseColor the color to display when state is false
     * @return an Animations instance for the boolean indicator
     */
    public Animations createAnimation(CANdle candle, LEDStrip strip, Supplier<Boolean> state, LEDColor trueColor, LEDColor falseColor) {
        return new BooleanAnim(candle, strip, state, trueColor, falseColor);
    }

    /**
     * Creates a countdown timer animation that visually displays remaining time.
     * 
     * <p>This animation starts a countdown from the specified time duration and progressively
     * turns off LEDs as time elapses. The strip starts fully lit and gradually dims from one
     * end to the other until all LEDs are off when time expires. Calling run() after completion
     * restarts the countdown.
     * 
     * @param candle the CANdle device to control
     * @param strip the LED strip segment to animate
     * @param time the countdown duration in seconds
     * @param color the color for the lit countdown LEDs
     * @return an Animations instance for the countdown timer
     */
    public Animations createAnimation(CANdle candle, LEDStrip strip, double time, LEDColor color) {
        return new Countdown(candle, strip, time, color);
    }

    /**
     * Creates a breathing animation that smoothly pulses between dim and full brightness.
     * 
     * <p>This animation creates a smooth sine wave breathing effect, oscillating between
     * a minimum brightness (dimmness) and full brightness at the specified frequency.
     * Multiple breathing animations can be phase-shifted to create wave effects across
     * different strip segments.
     * 
     * @param candle the CANdle device to control
     * @param strip the LED strip segment to animate
     * @param color the base color to breathe
     * @param frequency the breathing rate in Hz (cycles per second)
     * @param dimmness the minimum brightness level as a fraction (0.0-1.0, where 0.0 is off and 1.0 is full brightness)
     * @param phaseShift the phase offset in radians (0 to 2π) for synchronizing multiple breathing animations
     * @return an Animations instance for the breathing effect
     */
    public Animations createAnimation(CANdle candle, LEDStrip strip, LEDColor color, double frequency, double dimmness, double phaseShift) {
        return new Breathe(candle, strip, color, frequency, dimmness, phaseShift);
    }
    
    /**
     * Creates a command to set the entire LED strip to a solid color.
     * 
     * <p>This command sets all LEDs to the specified color. The command
     * completes instantly.
     * 
     * @param candle the CANdle device to control (must not be null)
     * @param color the color to set (must not be null)
     * @return an InstantCommand that sets the strip color
     * @throws IllegalArgumentException if any parameter is null
     */
    public Command setColor(CANdle candle, LEDColor color) {
        if (candle == null || color == null) {
            throw new IllegalArgumentException("Parameters cannot be null");
        }

        return new InstantCommand(() -> candle.setControl(
            new SolidColor(0, ledCount - 1)
                .withColor(new RGBWColor(color.getRed(), color.getGreen(), color.getBlue(), 0))
        ));
    }

    /**
     * Creates a command to set the entire LED strip to a solid color using RGB values.
     * 
     * <p>This command sets all LEDs to the specified color. The command
     * completes instantly. This is a convenience overload that accepts raw RGB
     * integer values instead of an LEDColor object.
     * 
     * @param candle the CANdle device to control (must not be null)
     * @param red the red channel value (0-255)
     * @param green the green channel value (0-255)
     * @param blue the blue channel value (0-255)
     * @return an InstantCommand that sets the strip color
     * @throws IllegalArgumentException if candle is null
     */
    public Command setColor(CANdle candle, int red, int green, int blue) {
        if (candle == null) {
            throw new IllegalArgumentException("CANdle cannot be null");
        }

        return new InstantCommand(() -> candle.setControl(
            new SolidColor(0, ledCount - 1)
                .withColor(new RGBWColor(red, green, blue, 0))
        ));
    }

    /**
     * Creates a command to set a specific LED strip segment to a solid color.
     * 
     * <p>This command sets only the LEDs in the specified strip segment to the
     * given color, leaving other LEDs unchanged. The command completes instantly.
     * 
     * @param candle the CANdle device to control (must not be null)
     * @param strip the LED strip segment to set (must not be null)
     * @param color the color to set (must not be null)
     * @return an InstantCommand that sets the strip segment color
     * @throws IllegalArgumentException if any parameter is null
     */
    public Command setStripColor(CANdle candle, LEDStrip strip, LEDColor color) {
        if (candle == null || strip == null || color == null) {
            throw new IllegalArgumentException("Parameters cannot be null");
        }

        return new InstantCommand(() -> candle.setControl(
            new SolidColor(strip.start, strip.end - 1)
                .withColor(new RGBWColor(color.getRed(), color.getGreen(), color.getBlue(), 0))
        ));
    }

    /**
     * Creates a command to set a specific LED strip segment to a solid color using RGB values.
     * 
     * <p>This command sets only the LEDs in the specified strip segment to the
     * given color, leaving other LEDs unchanged. The command completes instantly.
     * This is a convenience overload that accepts raw RGB integer values
     * instead of an LEDColor object.
     * 
     * @param candle the CANdle device to control (must not be null)
     * @param strip the LED strip segment to set (must not be null)
     * @param red the red channel value (0-255)
     * @param green the green channel value (0-255)
     * @param blue the blue channel value (0-255)
     * @return an InstantCommand that sets the strip segment color
     * @throws IllegalArgumentException if candle or strip is null
     */
    public Command setStripColor(CANdle candle, LEDStrip strip, int red, int green, int blue) {
        if (candle == null || strip == null) {
            throw new IllegalArgumentException("Parameters cannot be null");
        }
        
        return new InstantCommand(() -> candle.setControl(
            new SolidColor(strip.start, strip.end - 1)
                .withColor(new RGBWColor(red, green, blue, 0))
        ));
    }
    
    /**
     * Represents a contiguous segment of LEDs on a CANdle strip.
     * 
     * <p>LED strips can be logically divided into multiple segments for
     * independent control. Each segment is defined by a start index (inclusive)
     * and an end index (exclusive).
     * 
     * <p>Example:
     * <pre>
     * LEDStrip fullStrip = new LEDStrip(0, 60);     // 60 LEDs total
     * LEDStrip leftHalf = new LEDStrip(0, 30);      // First 30 LEDs
     * LEDStrip rightHalf = new LEDStrip(30, 60);    // Last 30 LEDs
     * </pre>
     * 
     * @param start the first LED index in the segment (inclusive)
     * @param end the LED index after the last LED in the segment (exclusive)
     */
    public record LEDStrip(
        int start,
        int end
    ){
        /**
         * Calculates the number of LEDs in this strip segment.
         * 
         * @return the length of the strip (end - start)
         */
        public int length() {
            return end - start;
        }
    }
    
    /**
     * Interface representing an LED color with red, green, and blue components.
     * 
     * <p>This interface allows for both predefined colors (via the Colors enum)
     * and custom RGB colors (via Colors.custom()).
     * 
     * @see Colors
     */
    public interface LEDColor {
        /**
         * Gets the red component of the color.
         * 
         * @return red value (0-255)
         */
        int getRed();
        
        /**
         * Gets the green component of the color.
         * 
         * @return green value (0-255)
         */
        int getGreen();
        
        /**
         * Gets the blue component of the color.
         * 
         * @return blue value (0-255)
         */
        int getBlue();
    }

    /**
     * Enumeration of predefined LED colors.
     * 
     * <p>Provides common colors for LED control. For custom colors, use
     * {@link #custom(int, int, int)}.
     * 
     * <p>Available colors:
     * <ul>
     *   <li>RED (255, 0, 0)</li>
     *   <li>GREEN (0, 255, 0)</li>
     *   <li>BLUE (0, 0, 255)</li>
     *   <li>YELLOW (255, 255, 0)</li>
     *   <li>PURPLE (128, 0, 128)</li>
     *   <li>ORANGE (255, 165, 0)</li>
     *   <li>WHITE (255, 255, 255)</li>
     *   <li>CYAN (0, 255, 255)</li>
     *   <li>MAGENTA (255, 0, 255)</li>
     *   <li>OFF (0, 0, 0) - turns LEDs off</li>
     * </ul>
     * 
     * <p>Example usage:
     * <pre>
     * LEDColor red = Colors.RED;
     * LEDColor custom = Colors.custom(100, 150, 200);
     * </pre>
     */
    public enum Colors implements LEDColor{
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 255, 0),
        PURPLE(128, 0, 128),
        ORANGE(255, 165, 0),
        WHITE(255, 255, 255),
        CYAN(0, 255, 255),
        MAGENTA(255, 0, 255),
        OFF(0, 0, 0);

        private final int redChannel;
        private final int greenChannel;
        private final int blueChannel;

        Colors(int red, int green, int blue) {
            redChannel = red;
            greenChannel = green;
            blueChannel = blue;
        }

        /**
         * Creates a custom LED color with the specified RGB values.
         * 
         * <p>This method allows creating colors that aren't in the predefined
         * Colors enum. The returned LEDColor can be used anywhere a color
         * is required.
         * 
         * @param red red component (0-255)
         * @param green green component (0-255)
         * @param blue blue component (0-255)
         * @return an LEDColor with the specified RGB values
         */
        public static LEDColor custom(int red, int green, int blue) {
            return new CustomColor(red, green, blue);
        }

        /**
         * Custom color implementation that allows arbitrary RGB values.
         * 
         * <p>This class is returned by {@link Colors#custom(int, int, int)} to create
         * colors not available in the predefined Colors enum.
         */
        public static class CustomColor implements LEDColor {
            public final int green;
            public final int blue;
            public final int red;
        
            /**
             * Constructs a custom color with the specified RGB values.
             * 
             * @param red red component (0-255)
             * @param green green component (0-255)
             * @param blue blue component (0-255)
             */
            public CustomColor(int red, int green, int blue) {
                this.red = red;
                this.green = green;
                this.blue = blue;
            }

            @Override
            public int getRed() {
                return red;
            }
            
            @Override
            public int getGreen() {
                return green;
            }
            
            @Override
            public int getBlue() {
                return blue;
            }
        }

        @Override
        public int getRed() {
            return redChannel;
        }
        
        @Override
        public int getGreen() {
            return greenChannel;
        }
        
        @Override
        public int getBlue() {
            return blueChannel;
        }
    }

    /**
     * Interface for LED animations that can be controlled through lifecycle methods.
     * 
     * <p>All animations implement this interface to provide consistent control over
     * starting, pausing, and stopping animations. Animations are backed by WPILib
     * Commands that are scheduled to the command scheduler.
     */
    public interface Animations {
        /**
         * Starts or resumes the animation.
         * 
         * <p>If the animation is not currently scheduled, this schedules it to the
         * command scheduler. If already running, this has no effect. For countdown
         * animations, calling run() after completion restarts the timer.
         */
        void run();
        
        /**
         * Pauses the animation while maintaining the current LED state.
         * 
         * <p>This cancels the underlying command but does not turn off the LEDs,
         * leaving them in their current state. Call run() to resume the animation.
         */
        void stop();
        
        /**
         * Stops the animation and turns off all LEDs in the strip.
         * 
         * <p>This cancels the underlying command and sets all LEDs in the animation's
         * strip segment to off (0, 0, 0). This provides a clean exit from the animation.
         */
        void end();

        /**
         * Checks if the animation is currently running.
         * 
         * @return A boolean indicating if the animation is active
         */
        boolean isRunning();
    }

    /**
     * State indicator animation that maps enum states to colors.
     * 
     * <p>This animation displays different colors based on the ordinal value of an enum state.
     * It continuously polls the state supplier and updates the LED strip when the state changes.
     * If there are more states than colors, the mapping wraps using modulo arithmetic.
     */
    private static class StateIndicator implements Animations{
        private CANdle candle;
        private LEDStrip strip;
        private Supplier<Enum<?>> states;
        private LEDColor[] colors;

        /**
         * Constructs a state indicator animation.
         * 
         * @param candle the CANdle device to control
         * @param strip the LED strip segment to animate
         * @param states supplier that returns the current enum state
         * @param colors array of colors mapped to state ordinal values
         */
        public StateIndicator(CANdle candle, LEDStrip strip, Supplier<Enum<?>> states, LEDColor... colors) {
            this.candle = candle;
            this.strip = strip;
            this.states = states;
            this.colors = colors;
        }

        private void draw() {
            if (states == null || colors == null || colors.length == 0) {
                candle.setControl(
                    new SolidColor(strip.start, strip.end - 1)
                        .withColor(new RGBWColor(0, 0, 0, 0))
                );
                return;
            }
        
            Enum<?> state = states.get();
            int colorIndex = state.ordinal() % colors.length;
            LEDColor color = colors[colorIndex];
        
            int r = color.getRed();
            int g = color.getGreen();
            int b = color.getBlue();
        
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(r, g, b, 0))
            );
        }

        private Command updateCommand = new Command() {
            @Override
            public void initialize() {}
    
            @Override
            public void execute() {
                draw();
            }
    
            @Override
            public boolean isFinished() {
                return false;
            }
    
            @Override
            public void end(boolean interrupted) {}
        };

        @Override
        public void run() {
            if (!updateCommand.isScheduled()) {
                updateCommand.schedule();
            }   
        }
    
        @Override
        public void stop() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
        }
    
        @Override
        public void end() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(0, 0, 0, 0))
            );
        }

        @Override
        public boolean isRunning() {
            return updateCommand.isScheduled();
        }
    }

    /**
     * Range value animation that displays a numeric value as a progress bar.
     * 
     * <p>This animation visualizes a value within a specified range by lighting a proportional
     * number of LEDs. The fill color represents the current value, while the empty color
     * shows the remaining capacity. The value is automatically clamped to the min/max range.
     */
    private static class RangeValue implements Animations {
        private final CANdle candle;
        private final CANdleLib.LEDStrip strip;
        private final double min;
        private final double max;
        private final DoubleSupplier valueSupplier;
        private final CANdleLib.LEDColor fillColor;
        private final CANdleLib.LEDColor emptyColor;
    
        /**
         * Constructs a range value animation.
         * 
         * @param candle the CANdle device to control
         * @param strip the LED strip segment to animate
         * @param min the minimum value of the range
         * @param max the maximum value of the range
         * @param valueSupplier supplier that returns the current value to display
         * @param fillColor color for LEDs representing the current value
         * @param emptyColor color for LEDs beyond the current value
         */
        public RangeValue(CANdle candle, CANdleLib.LEDStrip strip, double min, double max, DoubleSupplier valueSupplier, LEDColor fillColor, LEDColor emptyColor) {
            this.candle = candle;
            this.strip = strip;
            this.min = min;
            this.max = max;
            this.valueSupplier = valueSupplier;
            this.fillColor = fillColor;
            this.emptyColor = emptyColor;
        }
    
        private void draw() {
            double value = MathUtil.clamp(valueSupplier.getAsDouble(), min, max);
            int totalLEDs = strip.length();
            double fraction = (value - min) / (max - min);
            int litLEDs = (int) Math.round(fraction * totalLEDs);
    
            if (litLEDs > 0) {
                candle.setControl(
                    new SolidColor(strip.start, strip.start + litLEDs - 1)
                        .withColor(new RGBWColor(fillColor.getRed(), fillColor.getGreen(), fillColor.getBlue(), 0))
                );
            }
    
            int remaining = totalLEDs - litLEDs;
            if (remaining > 0) {
                candle.setControl(
                    new SolidColor(strip.start + litLEDs, strip.end - 1)
                        .withColor(new RGBWColor(emptyColor.getRed(), emptyColor.getGreen(), emptyColor.getBlue(), 0))
                );
            }
        }
    
        private Command updateCommand = new Command() {
            @Override
            public void initialize() {}
    
            @Override
            public void execute() {
                draw();
            }
    
            @Override
            public boolean isFinished() {
                return false;
            }
    
            @Override
            public void end(boolean interrupted) {}
        };
    
        @Override
        public void run() {
            if (!updateCommand.isScheduled()) {
                updateCommand.schedule();
            }   
        }
    
        @Override
        public void stop() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
        }
    
        @Override
        public void end() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(0, 0, 0, 0))
            );
        }

        @Override
        public boolean isRunning() {
            return updateCommand.isScheduled();
        }
    }    

    /**
     * Boolean indicator animation that displays one of two colors based on a boolean state.
     * 
     * <p>This animation switches between two colors depending on the boolean state. When true,
     * it displays trueColor; when false, it displays falseColor. Useful for binary indicators
     * like sensor states or system status.
     */
    private static class BooleanAnim implements Animations{
        private CANdle candle;
        private LEDStrip strip;
        private Supplier<Boolean> state;
        private LEDColor trueColor;
        private LEDColor falseColor;

        /**
         * Constructs a boolean indicator animation.
         * 
         * @param candle the CANdle device to control
         * @param strip the LED strip segment to animate
         * @param state supplier that returns the current boolean state
         * @param trueColor color to display when state is true
         * @param falseColor color to display when state is false
         */
        public BooleanAnim(CANdle candle, LEDStrip strip, Supplier<Boolean> state, LEDColor trueColor, LEDColor falseColor) {
            this.candle = candle;
            this.strip = strip;
            this.state = state;
            this.trueColor = trueColor;
            this.falseColor = falseColor;
        }

        private void draw() {
            LEDColor color = state.get() ? trueColor : falseColor;
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(color.getRed(), color.getGreen(), color.getBlue(), 0))
            );
        }

        private Command updateCommand = new Command() {
            @Override
            public void initialize() {}
    
            @Override
            public void execute() {
                draw();
            }
    
            @Override
            public boolean isFinished() {
                return false;
            }
    
            @Override
            public void end(boolean interrupted) {}
        };

        @Override
        public void run() {
            if (!updateCommand.isScheduled()) {
                updateCommand.schedule();
            }   
        }
    
        @Override
        public void stop() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
        }
    
        @Override
        public void end() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(0, 0, 0, 0))
            );
        }

        @Override
        public boolean isRunning() {
            return updateCommand.isScheduled();
        }
    }

    /**
     * Countdown timer animation that visually displays remaining time.
     * 
     * <p>This animation starts fully lit and progressively turns off LEDs as time elapses,
     * creating a visual countdown. When the timer expires, all LEDs are off and the animation
     * finishes. Calling run() after completion restarts the countdown from the beginning.
     */
    private static class Countdown implements Animations{
        private CANdle candle;
        private LEDStrip strip;
        private double time;
        private LEDColor color;
        private long startTimeMs;
        
        /**
         * Constructs a countdown timer animation.
         * 
         * @param candle the CANdle device to control
         * @param strip the LED strip segment to animate
         * @param time the countdown duration in seconds
         * @param color the color for the lit countdown LEDs
         */
        public Countdown(CANdle candle, LEDStrip strip, double time, LEDColor color) {
            this.candle = candle;
            this.strip = strip;
            this.time = time;
            this.color = color;
            this.startTimeMs = 0;
        }
        
        private void draw() {
            long currentTimeMs = System.currentTimeMillis();
            double elapsedSeconds = (currentTimeMs - startTimeMs) / 1000.0;
            double remaining = Math.max(0.0, time - elapsedSeconds);
            int totalLEDs = strip.length();
            double fraction = remaining / time;
            int litLEDs = (int) Math.round(fraction * totalLEDs);
            
            litLEDs = Math.max(0, Math.min(litLEDs, totalLEDs));
            
            if (litLEDs > 0) {
                candle.setControl(
                    new SolidColor(strip.start, strip.start + litLEDs - 1)
                        .withColor(new RGBWColor(color.getRed(), color.getGreen(), color.getBlue(), 0))
                );
            }
            
            int remainingLEDs = totalLEDs - litLEDs;
            if (remainingLEDs > 0) {
                candle.setControl(
                    new SolidColor(strip.start + litLEDs, strip.end - 1)
                        .withColor(new RGBWColor(0, 0, 0, 0))
                );
            }
        }
        
        private Command updateCommand = new Command() {
            @Override
            public void initialize() {
                startTimeMs = System.currentTimeMillis();
            }
    
            @Override
            public void execute() {
                draw();
            }
            
            @Override
            public boolean isFinished() {
                return (System.currentTimeMillis() - startTimeMs) / 1000.0 >= time;
            }
            
            @Override
            public void end(boolean interrupted) {}
        };

        @Override
        public void run() {
            if (!updateCommand.isScheduled()) {
                updateCommand.schedule();
            }
        }

        @Override
        public void stop() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
        }

        @Override
        public void end() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(0, 0, 0, 0))
            );
        }

        @Override
        public boolean isRunning() {
            return updateCommand.isScheduled();
        }
    }

    /**
     * Breathing animation that smoothly pulses between dim and full brightness.
     * 
     * <p>This animation creates a sine wave breathing effect, smoothly oscillating between
     * a minimum brightness (dimmness) and full brightness at the specified frequency. Multiple
     * breathing animations can be phase-shifted to create wave patterns across strip segments.
     */
    private static class Breathe implements Animations{
        private CANdle candle;
        private LEDStrip strip;
        private LEDColor color;
        private double frequency;
        private double dimmness;
        private double phaseShift;

        /**
         * Constructs a breathing animation.
         * 
         * @param candle the CANdle device to control
         * @param strip the LED strip segment to animate
         * @param color the base color to breathe
         * @param frequency the breathing rate in Hz (cycles per second)
         * @param dimmness the minimum brightness as a fraction (0.0-1.0)
         * @param phaseShift the phase offset in radians (0 to 2π)
         */
        public Breathe(CANdle candle, LEDStrip strip, LEDColor color, double frequency, double dimmness, double phaseShift) {
            this.candle = candle;
            this.strip = strip;
            this.color = color;
            this.frequency = frequency;
            this.dimmness = dimmness;
            this.phaseShift = phaseShift;
        }

        private void draw() {
            long currentTimeMs = System.currentTimeMillis();
            double periodMs = 1000.0 / frequency;
            double timeInPeriod = currentTimeMs % periodMs;
            
            double phase = (timeInPeriod / periodMs) * 2 * Math.PI + phaseShift;
            double brightness = (Math.sin(phase) + 1) / 2;
            
            double scale = dimmness + (brightness * (1.0 - dimmness));
            
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(
                        (int)(color.getRed() * scale), 
                        (int)(color.getGreen() * scale), 
                        (int)(color.getBlue() * scale), 
                        0
                    ))
            );
        }

        private Command updateCommand = new Command() {
            @Override
            public void initialize() {}
    
            @Override
            public void execute() {
                draw();
            }
    
            @Override
            public boolean isFinished() {
                return false;
            }
    
            @Override
            public void end(boolean interrupted) {}
        };

        @Override
        public void run() {
            if (!updateCommand.isScheduled()) {
                updateCommand.schedule();
            }
        }

        @Override
        public void stop() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
        }

        @Override
        public void end() {
            if (updateCommand.isScheduled()) {
                updateCommand.cancel();
            }
            candle.setControl(
                new SolidColor(strip.start, strip.end - 1)
                    .withColor(new RGBWColor(0, 0, 0, 0))
            );
        }

        @Override
        public boolean isRunning() {
            return updateCommand.isScheduled();
        }
    }
}