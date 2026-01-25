package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;

public class TurretConstants {
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    
    public static final int turretID = 0;
    public static final int absEncoderID = 0;
    public static final int hallEffectID = 0;

    public static final double positionConversionFactor = 0; // Example conversion factor
    public static final double velocityConversionFactor = 0; 

    public static final DCMotor turretGearbox = DCMotor.getNEO(0);
    public static final double turretMotorReduction = 0;

    public static final double absolutePositionOffsetRads = 0.0;
}
