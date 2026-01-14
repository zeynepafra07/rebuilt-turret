// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.led.patterns.LedPattern;

/** Manages the active LED pattern and handles timing logic. Ensures consistent animation updates based on time. */
public class AnimationController {
    private LedPattern activePattern;
    private int startIndex = 0;
    private final Timer timer = new Timer();

    /**
     * Sets a new pattern to be active. This resets the animation timer.
     *
     * @param pattern The pattern to display.
     * @param startIndex The index in the LED buffer where the pattern starts.
     */
    public void setPattern(LedPattern pattern, int startIndex) {
        this.activePattern = pattern;
        this.startIndex = startIndex;
        timer.restart();
        if (pattern != null) {
            pattern.initialize();
        }
    }

    /** Clears the current pattern. */
    public void clear() {
        this.activePattern = null;
        timer.stop();
        timer.reset();
    }

    /**
     * Updates the active pattern if one exists.
     *
     * @param buffer The LED buffer to write to.
     */
    public void update(AddressableLEDBuffer buffer) {
        if (activePattern != null) {
            activePattern.update(buffer, startIndex, timer.get());
        }
    }

    /**
     * Returns the name of the active pattern for logging.
     *
     * @return Name of the pattern class or "None".
     */
    public String getActivePatternName() {
        if (activePattern != null) {
            return activePattern.getClass().getSimpleName();
        }
        return "None";
    }
}
