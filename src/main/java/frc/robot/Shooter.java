package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;

public class Shooter {
    private SparkFlex       topMotor;
    private SparkBaseConfig topMotorConfig;

    private SparkFlex       bottomMotor;
    private SparkBaseConfig bottomMotorConfig;

    private RelativeEncoder topMotorEncoder;
    private RelativeEncoder bottomMotorEncoder;

    private PIDController topPIDController;
    private PIDController bottomPIDController;

    // Motor IDs
    private final int TOP_MOTOR_ID = 21;
    private final int BOTTOM_MOTOR_ID = 20;

    // PID Values
    private final double TOP_P = 0.01;
    private final double TOP_I = 0.0;
    private final double TOP_D = 0.0;

    private final double BOTTOM_P = 0.01;
    private final double BOTTOM_I = 0.0;
    private final double BOTTOM_D = 0.0;

    public Shooter() {
        topMotor       = new SparkFlex(TOP_MOTOR_ID, MotorType.kBrushless);
        topMotorConfig = new SparkFlexConfig();

        bottomMotor       = new SparkFlex(BOTTOM_MOTOR_ID, MotorType.kBrushless);
        bottomMotorConfig = new SparkFlexConfig();

        // TODO invert motor if needed
        topMotorConfig.idleMode(IdleMode.kCoast);
        topMotorConfig.inverted(true);

        bottomMotorConfig.idleMode(IdleMode.kCoast);
        bottomMotorConfig.inverted(false);

        topMotor.configure(topMotorConfig, 
                           ResetMode.kNoResetSafeParameters, 
                           PersistMode.kPersistParameters);

        bottomMotor.configure(bottomMotorConfig, 
                              ResetMode.kNoResetSafeParameters, 
                              PersistMode.kPersistParameters);

        topMotorEncoder =    topMotor.getEncoder();
        bottomMotorEncoder = bottomMotor.getEncoder();

        topPIDController =    new PIDController(TOP_P, TOP_I, TOP_D);
        bottomPIDController = new PIDController(BOTTOM_P, BOTTOM_I, BOTTOM_D);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setTopMotorVoltage(double voltage) {
        topMotor.setVoltage(voltage);
    }

    /**
     * voltage should be -12 to 12
     * @param voltage
     */
    public void setBottomMotorVoltage(double voltage) {
        bottomMotor.setVoltage(voltage);
    }

    /**
     * speed ....
     * @param speed
     */
    // TODO get conversion factor for speed
    public void setTopTargetSpeed(double speed) {
        
    }

    /**
     * speed ....
     * @param speed
     */
    // TODO get conversion factor for speed
    public void setBottomTargetSpeed(double speed) {

    }
}
