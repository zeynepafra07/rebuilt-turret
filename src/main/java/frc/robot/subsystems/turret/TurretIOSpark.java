package frc.robot.subsystems.turret;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.turret.TurretConstants.*;

public class TurretIOSpark implements TurretIO {
    //Hardware components
    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    private final AnalogInput absEncoder;
    private final DigitalInput hallEffect;

    private final SparkClosedLoopController turretController;

    private final Debouncer turretDebouncer = new Debouncer(0.5);

    public TurretIOSpark(){
        turretMotor = new SparkMax(TurretConstants.turretID, MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        absEncoder = new AnalogInput(TurretConstants.absEncoderID);
        hallEffect = new DigitalInput(TurretConstants.hallEffectID);

        turretController = turretMotor.getClosedLoopController();

        configure();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs){
        sparkStickyFault = false;
        ifOk(turretMotor, turretEncoder::getPosition, (value)-> inputs.positionRads = value);
        ifOk(turretMotor, turretEncoder::getVelocity, (value)-> inputs.velocityRadsPerSec = value);
        ifOk(turretMotor, new DoubleSupplier[] {turretMotor::getAppliedOutput, turretMotor::getBusVoltage}, 
            (values)-> inputs.appliedVolts = values[0] * values[1]);
        ifOk(turretMotor, turretMotor::getOutputCurrent, (value)-> inputs.supplyCurrentAmps = value);
        inputs.motorConnected = turretDebouncer.calculate(!sparkStickyFault);

        double voltage = absEncoder.getAverageVoltage();
        double ratio = voltage / RobotController.getVoltage5V();
        double absposition = ratio * 2.0 * Math.PI - TurretConstants.absolutePositionOffsetRads;
        inputs.absolutePositionRads = absposition;

        inputs.hallEffectTriggered = hallEffect.get();
    }

    @Override
    public void setVoltage(double voltage){
        turretMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double setpoint){
        turretController.setSetpoint(setpoint, ControlType.kPosition);
    }

    @Override
    public void resetEncoder(double absolutePositionRads){
        tryUntilOk(turretMotor, 5,() -> turretEncoder.setPosition(absolutePositionRads));
    }

    private void configure(){
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(TurretConstants.kP, 0, TurretConstants.kD).velocityFF(TurretConstants.kV);

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .apply(closedLoopConfig);
        turretConfig
            .encoder.positionConversionFactor(TurretConstants.positionConversionFactor)
            .velocityConversionFactor(TurretConstants.velocityConversionFactor);

        turretMotor.configure(turretConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}
