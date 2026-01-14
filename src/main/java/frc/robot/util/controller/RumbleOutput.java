package frc.robot.util.controller;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Hardware-agnostic interface for triggering haptic feedback. This allows subsystems to request rumble without needing
 * to know anything about the input HID or controller type.
 */
public interface RumbleOutput {
    /**
     * Set a constant rumble intensity.
     *
     * @param intensity Strength of rumble (0.0 to 1.0).
     */
    void setRumble(double intensity);

    /** Create a command that rumbles for a fixed duration. */
    Command rumbleOnTrue(double intensity, double seconds);

    /** Create a command that pulses the rumble. */
    Command rumblePulse(double intensity, double pulseDuration, int count);

    /** Stop all rumble. */
    default void stopRumble() {
        setRumble(0);
    }
}
