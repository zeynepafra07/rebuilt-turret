// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

/**
 * Senior-level controller utility for FRC.
 *
 * <p>Features: 1. Exclusive Multi-Press: Resolves conflicts between single and double clicks. 2. Simultaneous Chords:
 * Triggers based on multiple buttons pressed together. 3. Modifier Views: Shift-style button logic. 4. Decoupled
 * Rumble: Implements RumbleOutput for subsystem interaction.
 *
 * <p>Design Philosophy: All logic is evaluated inside BooleanSuppliers chained to Triggers. This ensures WPILib's
 * CommandScheduler handles the "update loop" for us.
 */
public class AdvancedController implements RumbleOutput {

    /** Unified mapping for the two most common FRC controllers. */
    public enum Button {
        kA(1, 2),
        kB(2, 3),
        kX(3, 1),
        kY(4, 4),
        kLB(5, 5),
        kRB(6, 6),
        kBack(7, 9),
        kStart(8, 10),
        kLeftStick(9, 11),
        kRightStick(10, 12);

        public final int xboxId;
        public final int ps4Id;

        Button(int xboxId, int ps4Id) {
            this.xboxId = xboxId;
            this.ps4Id = ps4Id;
        }
    }

    private final GenericHID m_hid;
    private final boolean m_isXbox;
    private final Map<Button, PressBinding> m_pressBindings = new HashMap<>();

    /** Factory for Xbox. */
    public static AdvancedController xbox(int port) {
        return new AdvancedController(new XboxController(port), true);
    }

    /** Factory for PS4. */
    public static AdvancedController ps4(int port) {
        return new AdvancedController(new PS4Controller(port), false);
    }

    private AdvancedController(GenericHID hid, boolean isXbox) {
        m_hid = hid;
        m_isXbox = isXbox;
    }

    private int getRawId(Button button) {
        return m_isXbox ? button.xboxId : button.ps4Id;
    }

    // --- Core Input Methods ---

    /** Standard non-exclusive trigger. Fires immediately. */
    public Trigger button(Button button) {
        return new Trigger(() -> m_hid.getRawButton(getRawId(button)));
    }

    /**
     * Creates an exclusive binding for a button.
     *
     * @param timeoutSeconds Window to wait for a second click before confirming a single click.
     */
    public PressBinding bindPress(Button button, double timeoutSeconds) {
        return m_pressBindings.computeIfAbsent(button, b -> new PressBinding(b, timeoutSeconds));
    }

    /**
     * Creates a chord (simultaneous press) trigger.
     *
     * @param tolerance How closely the buttons must be pressed (rising edges).
     */
    public Trigger chord(Button a, Button b, double tolerance) {
        int idA = getRawId(a);
        int idB = getRawId(b);

        return new Trigger(new java.util.function.BooleanSupplier() {
            private boolean lastA = false, lastB = false;
            private double riseA = -1, riseB = -1;

            @Override
            public boolean getAsBoolean() {
                boolean curA = m_hid.getRawButton(idA);
                boolean curB = m_hid.getRawButton(idB);
                double now = Timer.getFPGATimestamp();

                if (curA && !lastA) riseA = now;
                if (curB && !lastB) riseB = now;
                lastA = curA;
                lastB = curB;

                return curA && curB && Math.abs(riseA - riseB) < tolerance;
            }
        });
    }

    /** Returns a view that handles shift-style bindings using the provided modifier. */
    public ModifierView withModifier(Button modifier) {
        return new ModifierView(button(modifier));
    }

    // --- Exclusive Press Logic ---

    /** Manages IDLE -> WAITING -> RESOLVED states for a button. */
    public class PressBinding {
        private final int id;
        private final double window;
        private int count = 0;
        private double firstTime = -1;
        private boolean lastRaw = false;
        private double lastProcessed = -1;
        private boolean sPending = false, dPending = false;

        private PressBinding(Button b, double timeout) {
            this.id = getRawId(b);
            this.window = timeout;
        }

        private void update() {
            double now = Timer.getFPGATimestamp();
            if (now == lastProcessed) return;

            boolean raw = m_hid.getRawButton(id);
            boolean edge = raw && !lastRaw;
            lastRaw = raw;
            lastProcessed = now;

            if (edge) {
                if (count == 0 || (now - firstTime > window)) {
                    count = 1;
                    firstTime = now;
                } else if (count == 1) {
                    count = 0;
                    dPending = true;
                }
            }

            if (count == 1 && (now - firstTime > window)) {
                count = 0;
                sPending = true;
            }
        }

        public Trigger single() {
            return new Trigger(() -> {
                update();
                if (sPending) {
                    sPending = false;
                    return true;
                }
                return false;
            });
        }

        public Trigger doublePress() {
            return new Trigger(() -> {
                update();
                if (dPending) {
                    dPending = false;
                    return true;
                }
                return false;
            });
        }
    }

    // --- Modifier View ---

    public class ModifierView {
        private final Trigger modifier;

        private ModifierView(Trigger m) {
            this.modifier = m;
        }

        /** Fires if button is pressed WHILE modifier is held. */
        public Trigger press(Button b) {
            return button(b).and(modifier);
        }

        /** Fires if button is pressed WHILE modifier is NOT held. */
        public Trigger without(Button b) {
            return button(b).and(modifier.negate());
        }
    }

    // --- RumbleOutput Implementation ---

    @Override
    public void setRumble(double intensity) {
        m_hid.setRumble(RumbleType.kLeftRumble, intensity);
        m_hid.setRumble(RumbleType.kRightRumble, intensity);
    }

    @Override
    public Command rumbleOnTrue(double intensity, double seconds) {
        return Commands.startEnd(() -> setRumble(intensity), () -> setRumble(0)).withTimeout(seconds);
    }

    @Override
    public Command rumblePulse(double intensity, double duration, int count) {
        Command seq = Commands.none();
        for (int i = 0; i < count; i++) {
            seq = seq.andThen(rumbleOnTrue(intensity, duration)).andThen(Commands.waitSeconds(duration));
        }
        return seq;
    }

    /** Decoupled command generator for subsystem use. */
    public Command rumbleWhile(boolean condition, double intensity) {
        return Commands.runEnd(() -> setRumble(condition ? intensity : 0), () -> setRumble(0))
                .finallyDo(this::stopRumble);
    }
}
