package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Encapsula la configuración y el control de un módulo swerve:
 * <ul>
 *   <li>Motor de tracción (drive) en modo Motion Magic Velocity</li>
 *   <li>Motor de giro (steer/turning) en modo Motion Magic Expo (posición)</li>
 * </ul>
 *
 * Esta clase:
 * <ul>
 *   <li>Aplica las ganancias de los slots desde {@link SwerveConstants}</li>
 *   <li>Configura los parámetros de Motion Magic para ambos motores</li>
 *   <li>Provee métodos simples para setear velocidad lineal y ángulo</li>
 * </ul>
 */
public class SwerveController {

    // --- Motores físicos ---

    /** Motor de tracción del módulo. */
    private final TalonFX driveMotor;

    /** Motor de giro del módulo. */
    private final TalonFX turningMotor;

    // --- Configuración Phoenix 6 ---

    /** Configuración del TalonFX de tracción. */
    private final TalonFXConfiguration driveConfig;

    /** Configuración del TalonFX de giro. */
    private final TalonFXConfiguration turningConfig;

    // --- Demandos (requests) de control ---

    /**
     * Request de control para velocidad del motor de tracción.
     * Usa el Slot0 de la configuración.
     */
    private final VelocityVoltage velocityRequest;

    /**
     * Request de control para posición (Motion Magic Expo) del motor de giro.
     * Usa el Slot0 de la configuración.
     */
    private final MotionMagicExpoVoltage positionRequest;

    /**
     * Crea un controlador para un módulo swerve.
     *
     * @param driveMotor   TalonFX usado como drive (tracción)
     * @param turningMotor TalonFX usado como steer (giro)
     */
    public SwerveController(TalonFX driveMotor, TalonFX turningMotor) {
        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;

        // Instancia configuraciones vacías que luego llenamos con nuestras constantes.
        this.driveConfig = new TalonFXConfiguration();
        this.turningConfig = new TalonFXConfiguration();

        // Configura límites de corriente y modo neutral.
        configureMotors();

        // Requests de control iniciales (valor 0, slot 0).
        this.velocityRequest = new VelocityVoltage(0.0).withSlot(0);
        this.positionRequest = new MotionMagicExpoVoltage(0.0).withSlot(0);

        // Configura gains de slots y parámetros de Motion Magic.
        configureDriveGains();
        configureTurningGains();
        configureMotionMagic();

        // Aplica las configuraciones a los TalonFX.
        this.driveMotor.getConfigurator().apply(driveConfig);
        this.turningMotor.getConfigurator().apply(turningConfig);
    }

    // --------------------------------------------------------------------
    // CONFIGURACIÓN
    // --------------------------------------------------------------------
    
    /**
     * Configura los límites de corriente y el modo neutral de ambos motores.
     */
    private void configureMotors() {
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 120;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        turningConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turningConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turningConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turningConfig.CurrentLimits.StatorCurrentLimit = 120;
        turningConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    /**
     * Configura las ganancias del slot 0 del motor de tracción (velocidad).
     * <p>
     * Valores tomados de {@link SwerveConstants}:
     * kS, kV, kA, kP, kI, kD.
     */
    private void configureDriveGains() {
        var slot0 = driveConfig.Slot0;
        slot0.kS = SwerveConstants.VEL_KS;
        slot0.kV = SwerveConstants.VEL_KV;
        slot0.kA = SwerveConstants.VEL_KA;
        slot0.kP = SwerveConstants.VEL_KP;
        slot0.kI = SwerveConstants.VEL_KI;
        slot0.kD = SwerveConstants.VEL_KD;
    }

    /**
     * Configura las ganancias del slot 0 del motor de giro (posición).
     * <p>
     * Valores tomados de {@link SwerveConstants}:
     * kG, kS, kV, kA, kP, kI, kD.
     */
    private void configureTurningGains() {
        var slot0 = turningConfig.Slot0;
        slot0.kG = SwerveConstants.POS_KG;
        slot0.kS = SwerveConstants.POS_KS;
        slot0.kV = SwerveConstants.POS_KV;
        slot0.kA = SwerveConstants.POS_KA;
        slot0.kP = SwerveConstants.POS_KP;
        slot0.kI = SwerveConstants.POS_KI;
        slot0.kD = SwerveConstants.POS_KD;
    }

    /**
     * Configura los parámetros de Motion Magic para steer.
     * <ul>
     *   <li>Steer: vel. crucero, aceleración, jerk y parámetros Expo</li>
     * </ul>
     */
    private void configureMotionMagic() {
        // Motion Magic Expo en el motor de giro.
        var steerMM = turningConfig.MotionMagic;
        steerMM.MotionMagicCruiseVelocity = SwerveConstants.MAGIC_MOTION_VELOCITY_STR;
        steerMM.MotionMagicAcceleration = SwerveConstants.MAGIC_MOTION_ACCELERATION_STR;
        steerMM.MotionMagicJerk = SwerveConstants.MAGIC_MOTION_JERK_STR;
        steerMM.MotionMagicExpo_kV = SwerveConstants.MAGIC_MOTION_EXPO_KV_STR;
        steerMM.MotionMagicExpo_kA = SwerveConstants.MAGIC_MOTION_EXPO_KA_STR;
    }

    // --------------------------------------------------------------------
    // COMANDOS
    // --------------------------------------------------------------------

    /**
     * Establece la velocidad lineal deseada del módulo.
     *
     * @param velocityMps velocidad objetivo en metros por segundo.
     *                    Internamente se convierte a unidades de rotaciones
     *                    de motor por segundo usando {@link SwerveConstants#ROT_2_M}.
     */
    public void setVelocity(double velocityMps) {
        // Convierte de m/s a rotaciones de motor por "unidad de tiempo" de Phoenix.
        if (Math.abs(velocityMps) > 0.1) {
            double motorVelocity = velocityMps / SwerveConstants.ROT_2_M;
            driveMotor.setControl(velocityRequest.withVelocity(motorVelocity));
        } else {
            driveMotor.stopMotor();
        }
    }

    /**
     * Establece el ángulo deseado del módulo.
     *
     * @param angleRad ángulo objetivo en radianes.
     *                 Se convierte a rotaciones del motor de giro usando
     *                 {@link SwerveConstants#ROT_2_RAD}.
     */
    public void setAngle(double angleRad) {
        if (Math.abs(angleRad - turningMotor.getPosition().getValueAsDouble()) > Math.toRadians(1)){
            // Convierte de radianes a rotaciones del eje (considerando relación de transmisión).
            double rotations = angleRad / SwerveConstants.ROT_2_RAD;
            turningMotor.setControl(positionRequest.withPosition(rotations));
        } else {
            turningMotor.stopMotor();
        }
    }
}
