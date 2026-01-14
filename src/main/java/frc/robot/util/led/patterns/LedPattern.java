// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.led.patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Random;

/**
 * Abstract base class for all LED patterns. Patterns are responsible for updating a specific segment of the LED buffer
 * based on time.
 */
public abstract class LedPattern {
    /** The number of LEDs in this pattern. */
    protected int ledCount = 0;

    /** The brightness of the pattern (0.0 to 1.0). */
    protected double brightness = 1.0;

    /** The speed of the animation (arbitrary units, typically 0.0 to 1.0 or Hz). */
    protected double speed = 1.0;

    /** Called when the pattern is set to active. Use this to reset any internal state. */
    public void initialize() {}

    /**
     * Updates the LED buffer for this pattern.
     *
     * @param buffer The LED buffer to write to.
     * @param startIndex The starting index in the buffer for this pattern.
     * @param timeSeconds The elapsed time since the animation started.
     */
    public abstract void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds);

    /**
     * Checks if the animation is finished.
     *
     * @return true if the animation is complete (for one-shot animations).
     */
    public boolean isFinished() {
        return false;
    }

    /**
     * Sets the brightness of the pattern.
     *
     * @param brightness Brightness from 0.0 to 1.0
     */
    public void setBrightness(double brightness) {
        this.brightness = Math.max(0.0, Math.min(1.0, brightness));
    }

    /**
     * Sets the speed of the pattern.
     *
     * @param speed Animation speed.
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * Sets the number of LEDs this pattern controls.
     *
     * @param count Number of LEDs.
     */
    public void setLedCount(int count) {
        this.ledCount = count;
    }

    // =========================================================================
    // INNER CLASSES
    // =========================================================================

    /** A simple pattern that sets a static color. */
    public static class SolidColor extends LedPattern {
        private final Color color;

        public SolidColor(Color color) {
            this.color = color;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    buffer.setLED(startIndex + i, color);
                }
            }
        }
    }

    /** Pattern that blinks a color on and off at a specified interval. */
    public static class Strobe extends LedPattern {
        private final Color color;
        private final double duration;

        public Strobe(Color color, double durationSeconds) {
            this.color = color;
            this.duration = durationSeconds;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            boolean on = (timeSeconds % duration) < (duration / 2.0);
            Color targetColor = on ? color : Color.kBlack;

            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    buffer.setLED(startIndex + i, targetColor);
                }
            }
        }
    }

    /** Pattern that simulates a breathing effect by modulating brightness with a sine wave. */
    public static class Breathe extends LedPattern {
        private final Color color;
        private final double period;

        public Breathe(Color color, double periodSeconds) {
            this.color = color;
            this.period = periodSeconds;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            double brightnessFactor = (Math.sin(timeSeconds * (2 * Math.PI) / period) + 1.0) / 2.0;
            brightnessFactor *= this.brightness;

            int r = (int) (color.red * brightnessFactor * 255);
            int g = (int) (color.green * brightnessFactor * 255);
            int b = (int) (color.blue * brightnessFactor * 255);

            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    buffer.setRGB(startIndex + i, r, g, b);
                }
            }
        }
    }

    /** Pattern that displays a moving rainbow effect. */
    public static class Rainbow extends LedPattern {
        public Rainbow(double speedScale) {
            this.speed = speedScale;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            double hueOffset = (timeSeconds * speed * 180.0) % 180.0;
            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    double hue = (hueOffset + (i * 180.0 / 60.0)) % 180.0;
                    buffer.setHSV(startIndex + i, (int) hue, 255, (int) (255 * brightness));
                }
            }
        }
    }

    /** Pattern simulating the 'Knight Rider' scanner effect. */
    public static class Larson extends LedPattern {
        private final Color color;
        private final int eyeSize;

        public Larson(Color color, int eyeSize, double speedScale) {
            this.color = color;
            this.eyeSize = eyeSize;
            this.speed = speedScale;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            double t = timeSeconds * speed;
            double pos = (Math.sin(t * Math.PI) + 1.0) / 2.0 * (ledCount - 1);

            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    double dist = Math.abs(i - pos);
                    double factor = Math.max(0, 1.0 - (dist / (double) eyeSize));
                    factor = Math.pow(factor, 2);
                    factor *= brightness;

                    int r = (int) (color.red * factor * 255);
                    int g = (int) (color.green * factor * 255);
                    int b = (int) (color.blue * factor * 255);

                    buffer.setRGB(startIndex + i, r, g, b);
                }
            }
        }
    }

    /** Pattern simulating fire with cooling and sparking effects. */
    public static class Fire extends LedPattern {
        private final int cooling;
        private final int sparking;
        private final boolean reverse;
        private int[] heat;
        private final Random random = new Random();
        private double lastUpdateTime = 0;

        public Fire(double speedScale, int cooling, int sparking, int length) {
            this.speed = speedScale;
            this.cooling = cooling;
            this.sparking = sparking;
            this.reverse = false;
            this.heat = new int[length];
            this.ledCount = length;
        }

        @Override
        public void setLedCount(int count) {
            super.setLedCount(count);
            // Re-allocate heat array if size changes
            if (this.heat.length != count) {
                this.heat = new int[count];
            }
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            if (timeSeconds - lastUpdateTime > (0.05 / speed)) {
                stepFire();
                lastUpdateTime = timeSeconds;
            }

            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    Color c = heatToColor(heat[i]);
                    int r = (int) (c.red * brightness * 255);
                    int g = (int) (c.green * brightness * 255);
                    int b = (int) (c.blue * brightness * 255);

                    int index = reverse ? (ledCount - 1 - i) : i;
                    buffer.setRGB(startIndex + index, r, g, b);
                }
            }
        }

        private void stepFire() {
            for (int i = 0; i < ledCount; i++) {
                int cool = random.nextInt(cooling);
                if (cool > heat[i]) {
                    heat[i] = 0;
                } else {
                    heat[i] -= cool;
                }
            }

            for (int k = ledCount - 1; k >= 3; k--) {
                heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
            }

            if (random.nextInt(255) < sparking) {
                int y = random.nextInt(7);
                if (y < ledCount) {
                    heat[y] = Math.min(255, heat[y] + random.nextInt(160) + 160);
                }
            }
        }

        private Color heatToColor(int temperature) {
            int t192 = (int) Math.round((temperature / 255.0) * 191);
            int heatramp = t192 & 0x3F;
            heatramp <<= 2;

            if (t192 > 0x80) {
                return new Color(255, 255, heatramp);
            } else if (t192 > 0x40) {
                return new Color(255, heatramp, 0);
            } else {
                return new Color(heatramp, 0, 0);
            }
        }
    }

    /** Pattern that makes random LEDs twinkle on and off. */
    public static class Twinkle extends LedPattern {
        private final Color color;
        private final double twinkleChance;
        private final Random random = new Random();
        private double lastUpdateTime = 0;

        public Twinkle(Color color, double twinkleChance, double speedScale) {
            this.color = color;
            this.twinkleChance = twinkleChance;
            this.speed = speedScale;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            if (timeSeconds - lastUpdateTime > (0.1 / speed)) {
                lastUpdateTime = timeSeconds;
                for (int i = 0; i < ledCount; i++) {
                    if (startIndex + i < buffer.getLength()) {
                        boolean on = random.nextDouble() < twinkleChance;
                        if (on) {
                            int r = (int) (color.red * brightness * 255);
                            int g = (int) (color.green * brightness * 255);
                            int b = (int) (color.blue * brightness * 255);
                            buffer.setRGB(startIndex + i, r, g, b);
                        } else {
                            buffer.setRGB(startIndex + i, 0, 0, 0);
                        }
                    }
                }
            }
        }
    }

    /** Variation of TwinklePattern where LEDs start on and randomly turn off. */
    public static class TwinkleOff extends LedPattern {
        private final Color color;
        private final double twinkleChance;
        private final Random random = new Random();
        private double lastUpdateTime = 0;

        public TwinkleOff(Color color, double twinkleChance, double speedScale) {
            this.color = color;
            this.twinkleChance = twinkleChance;
            this.speed = speedScale;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            if (timeSeconds - lastUpdateTime > (0.1 / speed)) {
                lastUpdateTime = timeSeconds;
            } else {
                return;
            }

            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    boolean off = random.nextDouble() < twinkleChance;
                    if (!off) {
                        int r = (int) (color.red * brightness * 255);
                        int g = (int) (color.green * brightness * 255);
                        int b = (int) (color.blue * brightness * 255);
                        buffer.setRGB(startIndex + i, r, g, b);
                    } else {
                        buffer.setRGB(startIndex + i, 0, 0, 0);
                    }
                }
            }
        }
    }

    /** Pattern that makes a single color fade in and out. */
    public static class SingleFade extends LedPattern {
        private final Color color;
        private final double period;

        public SingleFade(Color color, double periodSeconds) {
            this.color = color;
            this.period = periodSeconds;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            double factor = (Math.sin(timeSeconds * (2 * Math.PI) / period) + 1.0) / 2.0;
            factor = Math.pow(factor, 2.0);
            factor *= brightness;

            int r = (int) (color.red * factor * 255);
            int g = (int) (color.green * factor * 255);
            int b = (int) (color.blue * factor * 255);

            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    buffer.setRGB(startIndex + i, r, g, b);
                }
            }
        }
    }

    /** Pattern that shifts a color across the LED strip in a specified direction. */
    public static class ColorFlow extends LedPattern {
        private final Color color;
        private final Direction direction;

        public enum Direction {
            FORWARD,
            BACKWARD
        }

        public ColorFlow(Color color, Direction direction, double speedScale) {
            this.color = color;
            this.direction = direction;
            this.speed = speedScale;
        }

        @Override
        public void update(AddressableLEDBuffer buffer, int startIndex, double timeSeconds) {
            double offset = timeSeconds * speed * ledCount;
            for (int i = 0; i < ledCount; i++) {
                if (startIndex + i < buffer.getLength()) {
                    double pos = (direction == Direction.FORWARD) ? (i - offset) : (i + offset);
                    double cycle = ledCount / 2.0;
                    double factor = (Math.sin(pos * (2 * Math.PI) / cycle) + 1.0) / 2.0;

                    int r = (int) (color.red * factor * brightness * 255);
                    int g = (int) (color.green * factor * brightness * 255);
                    int b = (int) (color.blue * factor * brightness * 255);

                    buffer.setRGB(startIndex + i, r, g, b);
                }
            }
        }
    }
}
