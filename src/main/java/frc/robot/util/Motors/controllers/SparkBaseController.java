package frc.robot.util.motors.controllers;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.motors.MotorIO;

public class SparkBaseController extends SparkBase implements MotorIO {

    SparkBaseConfig sparkBaseConfig;
    private final ControllerType controllerType;

    /**
     * Setups the motor..
     *
     * @param motor The {@link Motor} dataclass containing motor configuration details
     * @return the {@code MotorConfigurator} instance for chaining
     * @throws IllegalArgumentException if the motor type is unsupported
     */
    public SparkBaseController(Motor motor) {
        super(
                motor.canID(),
                MotorType.kBrushless,
                motor.controllerType() == ControllerType.SPARKMAX ? SparkModel.SparkMax : SparkModel.SparkFlex);
        this.controllerType = motor.controllerType();
    }

    public void isInverted(boolean isInverted) {
        if (controllerType == ControllerType.SPARKMAX) {
            sparkBaseConfig = new SparkMaxConfig();
        } else {
            sparkBaseConfig = new SparkFlexConfig();
        }
        sparkBaseConfig.inverted(isInverted);

        super.configure(sparkBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
