package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.util.controller.AdvancedController;
import frc.robot.util.controller.AdvancedController.Button;

/** Example class demonstrating senior-level usage of the AdvancedController. */
public class ControllerExamples {

    // 1. Initialize controllers
    private final AdvancedController driver = AdvancedController.xbox(0);
    private final AdvancedController operator = AdvancedController.ps4(1);

    public void configureBindings() {

        // --- SCENARIO 1: Exclusive Multi-Press ---
        // Goal: A single button does two different things without conflict.
        var xBinding = driver.bindPress(Button.kX, 0.3);

        // Single press toggle (Fires after 300ms if no second click)
        xBinding.single().onTrue(new InstantCommand(() -> System.out.println("Low Gear Set")));

        // Double press (Fires immediately on second click, cancels single)
        // Also triggers a double-pulse rumble for tactile confirmation.
        xBinding.doublePress()
                .onTrue(new InstantCommand(() -> System.out.println("High Gear Set"))
                        .alongWith(driver.rumblePulse(1.0, 0.1, 2)));

        // --- SCENARIO 2: Chord (Simultaneous Press) ---
        // Goal: Emergency reset requires two hands to prevent accidental activation.
        driver.chord(Button.kLB, Button.kRB, 0.1)
                .onTrue(new PrintCommand("!! EMERGENCY RESET !!").alongWith(driver.rumbleOnTrue(1.0, 0.5)));

        // --- SCENARIO 3: Modifier (Shift) Logic ---
        // Goal: Multiply the number of available functions using a "Shift" button.
        var shifted = operator.withModifier(Button.kLB); // L1 is the shift button

        // L1 + Square = Intake Reverse
        shifted.press(Button.kX).whileTrue(new RunCommand(() -> System.out.println("Intake Backwards")));

        // Square (without L1) = Intake Forward
        // EXPLANATION: We use .without() to ensure this command ONLY runs if L1 is NOT
        // held.
        // This prevents both "Intake Forward" and "Intake Backwards" from running at
        // the same time.
        shifted.without(Button.kX).whileTrue(new RunCommand(() -> System.out.println("Intake Forwards")));

        // --- SCENARIO 4: Haptic Feedback (Rumble) ---
        // Goal: Notify driver when a sensor detects a game piece.
        // (Assuming a sensor Trigger exists elsewhere)
        // pieceDetectedTrigger.onTrue(driver.rumblePulse(0.7, 0.15, 1));

        // Example: Persistent rumble while a certain mode is active
        driver.button(Button.kY).whileTrue(driver.rumbleWhile(true, 0.3));
    }
}
