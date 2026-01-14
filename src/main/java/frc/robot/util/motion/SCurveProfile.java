package frc.robot.util.motion;

/**
 * A jerk-limited S-curve motion profile implementation, equivalent to Phoenix 6 Motion Magic.
 *
 * <p>Jerk limiting improves mechanical longevity and control stability by: 1. Reducing mechanical stress: Smooth
 * acceleration ramps prevent shock loads on drivetrains. 2. Reducing vibration: High-frequency oscillations are
 * minimized by limiting the rate of change of acceleration. 3. Improving tracking: PID loops can track smooth reference
 * curves more accurately than trapezoidal ones.
 *
 * <p>The 7-Phase S-Curve Logic: Phase 1: Jerk Up (+J) - Accel ramps to aMax. Phase 2: Constant Accel (+aMax) - Velocity
 * ramps linearly. Phase 3: Jerk Down (-J) - Accel ramps to 0. Velocity hits peak. Phase 4: Cruise (0 Accel) - Constant
 * velocity. Phase 5: Jerk Down (-J) - Accel ramps to -aMax. Phase 6: Constant Decel (-aMax) - Velocity ramps down
 * linearly. Phase 7: Jerk Up (+J) - Accel ramps to 0. Velocity hits 0 at target.
 */
public class SCurveProfile {
    public static class State {
        public double p, v, a;

        public State(double p, double v, double a) {
            this.p = p;
            this.v = v;
            this.a = a;
        }
    }

    private final double m_vMax;
    private final double m_aMax;
    private final double m_jMax;

    private State m_current = new State(0, 0, 0);
    private State m_goal = new State(0, 0, 0);
    private State m_initialState = new State(0, 0, 0);

    // Phase timings
    private double t1, t2, t3, t4, t5, t6, t7;
    private double m_elapsedTime;
    private boolean m_isFlipped = false;

    public SCurveProfile(double vMax, double aMax, double jMax) {
        m_vMax = vMax;
        m_aMax = aMax;
        m_jMax = jMax;
    }

    public void reset(double p, double v) {
        m_current = new State(p, v, 0);
        m_initialState = new State(p, v, 0);
        m_elapsedTime = 0;
        t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0;
    }

    public void setGoal(double targetP) {
        m_goal = new State(targetP, 0, 0);
        replan();
    }

    private void replan() {
        m_initialState = new State(m_current.p, m_current.v, m_current.a);
        m_elapsedTime = 0;

        double deltaP = m_goal.p - m_initialState.p;
        m_isFlipped = deltaP < 0;
        double s = Math.abs(deltaP);

        // Standard 7-phase calculation
        double j = m_jMax;
        double a = m_aMax;
        double v = m_vMax;

        // Can we reach aMax and still hit exactly vMax?
        // Minimum velocity to reach aMax: v_min_for_aMax = aMax^2 / j
        if (v * j < a * a) {
            a = Math.sqrt(v * j);
        }

        double tj = a / j; // Time to ramp jerk to a
        double tv = (v - (a * tj)) / a; // Time at constant acceleration a

        // Distance covered during acceleration and deceleration (symmetric)
        // distance_accel = vMax * (tj + tv + tj) / 2
        // distance_decel = vMax * (tj + tv + tj) / 2
        double s_crit = v * (2 * tj + tv);

        if (s < s_crit) {
            // Short move: We don't reach vMax. Solve for vPeak.
            // s = vPeak * (2*tj + tv_new). If we reach aMax:
            // vPeak = a * (tj + tv_new).
            // s = a * (tj + tv_new) * (2*tj + tv_new) = a * (2*tj^2 + 3*tj*tv_new +
            // tv_new^2)
            double disc = 9 * tj * tj - 4 * (2 * tj * tj - s / a);
            if (disc >= 0) {
                tv = (-3 * tj + Math.sqrt(disc)) / 2.0;
            } else {
                // We don't reach aMax either. s = 2 * j * tj^3.
                tj = Math.pow(s / (2.0 * j), 1.0 / 3.0);
                tv = 0;
                a = j * tj;
            }
            v = a * (tj + tv);
            t4 = 0;
        } else {
            // Long move: Cruise at vMax
            t4 = (s - s_crit) / v;
        }

        t1 = t3 = t5 = t7 = tj;
        t2 = t6 = tv;
    }

    public State update(double dt) {
        m_elapsedTime += dt;
        double t = m_elapsedTime;

        double j = 0, currA = 0, currV = 0, currP = 0;

        double b1 = t1;
        double b2 = b1 + t2;
        double b3 = b2 + t3;
        double b4 = b3 + t4;
        double b5 = b4 + t5;
        double b6 = b5 + t6;
        double b7 = b6 + t7;

        double v1 = 0.5 * m_jMax * t1 * t1;
        double v2 = v1 + (m_jMax * t1) * t2;
        double v3 = v2 + (m_jMax * t1) * t3 - 0.5 * m_jMax * t3 * t3;
        double p1 = (1.0 / 6.0) * m_jMax * Math.pow(t1, 3);
        double p2 = p1 + v1 * t2 + 0.5 * (m_jMax * t1) * t2 * t2;
        double p3 = p2 + v2 * t3 + 0.5 * (m_jMax * t1) * t3 * t3 - (1.0 / 6.0) * m_jMax * Math.pow(t3, 3);
        double p4 = p3 + v3 * t4;
        double p5 = p4 + v3 * t5 - (1.0 / 6.0) * m_jMax * Math.pow(t5, 3);
        double p6 = p5 + (v3 - 0.5 * m_jMax * t5 * t5) * t6 - 0.5 * (m_jMax * t5) * t6 * t6;

        if (t <= b1) {
            j = m_jMax;
            currA = j * t;
            currV = 0.5 * j * t * t;
            currP = (1.0 / 6.0) * j * Math.pow(t, 3);
        } else if (t <= b2) {
            double dtP = t - b1;
            j = 0;
            currA = m_jMax * t1;
            currV = v1 + currA * dtP;
            currP = p1 + v1 * dtP + 0.5 * currA * dtP * dtP;
        } else if (t <= b3) {
            double dtP = t - b2;
            j = -m_jMax;
            currA = (m_jMax * t1) - m_jMax * dtP;
            currV = v2 + (m_jMax * t1) * dtP - 0.5 * m_jMax * dtP * dtP;
            currP = p2 + v2 * dtP + 0.5 * (m_jMax * t1) * dtP * dtP - (1.0 / 6.0) * m_jMax * Math.pow(dtP, 3);
        } else if (t <= b4) {
            double dtP = t - b3;
            j = 0;
            currA = 0;
            currV = v3;
            currP = p3 + v3 * dtP;
        } else if (t <= b5) {
            double dtP = t - b4;
            j = -m_jMax;
            currA = -m_jMax * dtP;
            currV = v3 - 0.5 * m_jMax * dtP * dtP;
            currP = p4 + v3 * dtP - (1.0 / 6.0) * m_jMax * Math.pow(dtP, 3);
        } else if (t <= b6) {
            double dtP = t - b5;
            j = 0;
            currA = -m_jMax * t5;
            currV = (v3 - 0.5 * m_jMax * t5 * t5) + currA * dtP;
            currP = p5 + (v3 - 0.5 * m_jMax * t5 * t5) * dtP + 0.5 * currA * dtP * dtP;
        } else if (t <= b7) {
            double dtP = t - b6;
            j = m_jMax;
            currA = (-m_jMax * t5) + m_jMax * dtP;
            double v6 = (v3 - 0.5 * m_jMax * t5 * t5) - (m_jMax * t5) * t6;
            currV = v6 - (m_jMax * t5) * dtP + 0.5 * m_jMax * dtP * dtP;
            currP = p6 + v6 * dtP - 0.5 * (m_jMax * t5) * dtP * dtP + (1.0 / 6.0) * m_jMax * Math.pow(dtP, 3);
        } else {
            j = 0;
            currA = 0;
            currV = 0;
            currP = Math.abs(m_goal.p - m_initialState.p);
        }

        m_current = new State(
                m_initialState.p + (m_isFlipped ? -currP : currP),
                m_isFlipped ? -currV : currV,
                m_isFlipped ? -currA : currA);
        return m_current;
    }

    public boolean isFinished() {
        return m_elapsedTime >= (t1 + t2 + t3 + t4 + t5 + t6 + t7);
    }

    public State getCurrentState() {
        return m_current;
    }
}
