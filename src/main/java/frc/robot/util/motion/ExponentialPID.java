// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.motion;

import edu.wpi.first.math.MathUtil;

/**
 * A manual implementation of an Advanced Exponential PID controller for WPILib.
 *
 * <p>Design Rationale: - Exponential scaling is applied ONLY to the Proportional (P) term. - Why? The P term provides
 * the immediate response to error. Exponential scaling here allows for aggressive correction of large errors while
 * remaining extremely smooth and precise near the setpoint. - Why not I or D? The Integral (I) term is for steady-state
 * error correction; non-linear integration (exponential) can cause the accumulator to balloon or shrink unpredictably,
 * leading to limit cycles. The Derivative (D) term provides damping; non-linear damping can cause erratic oscillations
 * during high-speed transitions.
 */
public class ExponentialPID {
    private double m_kp;
    private double m_ki;
    private double m_kd;
    private double m_exponent;

    private double m_setpoint;
    private double m_prevMeasurement;
    private double m_totalError;

    private double m_iMin = -1.0;
    private double m_iMax = 1.0;

    // FRC standard loop period is 20ms
    private static final double kPeriod = 0.020;
    private boolean m_firstRun = true;

    /**
     * Constructs a new Advanced Exponential PID controller.
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param exponent Exponent for the P term (1.0 = linear PID, >1.0 = exponential)
     */
    public ExponentialPID(double kp, double ki, double kd, double exponent) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
        m_exponent = exponent;
        reset();
    }

    /**
     * Sets the desired target value.
     *
     * @param setpoint The target setpoint
     */
    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    /**
     * Resets the controller state (integrator and derivative memory). Should be called when the mechanism starts a new
     * motion.
     */
    public void reset() {
        m_totalError = 0;
        m_prevMeasurement = 0;
        m_firstRun = true;
    }

    /**
     * Configures the range of the internal integrator to prevent windup.
     *
     * @param min Minimum accumulation
     * @param max Maximum accumulation
     */
    public void setIntegratorRange(double min, double max) {
        m_iMin = min;
        m_iMax = max;
    }

    /**
     * Calculates the control output based on the current measurement.
     *
     * @param measurement The current process variable (e.g., encoder value)
     * @return Output clamped to [-1.0, 1.0]
     */
    public double calculate(double measurement) {
        double error = m_setpoint - measurement;

        // 1. Proportional Term (Exponentially Scaled)
        // Preserve sign: sign(error) * |error|^exponent
        double pOutput = Math.copySign(Math.pow(Math.abs(error), m_exponent), error) * m_kp;

        // 2. Integral Term (Linear Error)
        // Accumulate error over time: Integral = sum(error * dt)
        m_totalError += error * kPeriod;
        // Anti-Windup: Clamp the accumulator
        m_totalError = MathUtil.clamp(m_totalError, m_iMin, m_iMax);
        double iOutput = m_totalError * m_ki;

        // 3. Derivative Term (Linear, using Derivative-on-Measurement)
        // We use (prev - current) / dt instead of (currentError - prevError) / dt
        // to avoid "derivative kick" when the setpoint changes.
        double dOutput = 0;
        if (!m_firstRun) {
            double dMeasurement = (measurement - m_prevMeasurement) / kPeriod;
            dOutput = -dMeasurement * m_kd;
        }

        // Update state for next cycle
        m_prevMeasurement = measurement;
        m_firstRun = false;

        // Sum components and clamp to motor controller range
        return MathUtil.clamp(pOutput + iOutput + dOutput, -1.0, 1.0);
    }

    // Getters and Setters for tuning
    public void setP(double kp) {
        m_kp = kp;
    }

    public void setI(double ki) {
        m_ki = ki;
    }

    public void setD(double kd) {
        m_kd = kd;
    }

    public void setExponent(double exponent) {
        m_exponent = exponent;
    }

    public double getP() {
        return m_kp;
    }

    public double getI() {
        return m_ki;
    }

    public double getD() {
        return m_kd;
    }

    public double getExponent() {
        return m_exponent;
    }

    public double getSetpoint() {
        return m_setpoint;
    }
}
