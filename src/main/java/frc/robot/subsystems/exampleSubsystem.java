// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.Led;
import frc.robot.util.led.patterns.LedPattern;

public class exampleSubsystem extends SubsystemBase {

    public exampleSubsystem(Led led) {
        led.setAnimation(
                new LedPattern.ColorFlow(Color.kAliceBlue, LedPattern.ColorFlow.Direction.FORWARD, 1.0), 0, 11);
    }
}
