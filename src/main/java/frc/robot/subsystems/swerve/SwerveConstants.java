package frc.robot.subsystems.swerve;

/**
 * Conjunto de constantes específicas del sistema swerve.
 *
 * Solo se incluyen las que realmente se usan en las clases actuales
 * (módulos, controlador, IO, etc.).
 */
public final class SwerveConstants {

    private SwerveConstants() {
        // Clase de solo constantes: no instanciable.
    }

    // --------------------------------------------------------------------
    // IDs DE DISPOSITIVOS (CTR)
    // --------------------------------------------------------------------

    /** CAN ID del motor de tracción del módulo delantero izquierdo. */
    public static final int FL_PWR = 4;
    /** CAN ID del motor de tracción del módulo delantero derecho. */
    public static final int FR_PWR = 3;
    /** CAN ID del motor de tracción del módulo trasero izquierdo. */
    public static final int BL_PWR = 2;
    /** CAN ID del motor de tracción del módulo trasero derecho. */
    public static final int BR_PWR = 1;

    /** CAN ID del motor de giro del módulo delantero izquierdo. */
    public static final int FL_STR = 8;
    /** CAN ID del motor de giro del módulo delantero derecho. */
    public static final int FR_STR = 7;
    /** CAN ID del motor de giro del módulo trasero izquierdo. */
    public static final int BL_STR = 6;
    /** CAN ID del motor de giro del módulo trasero derecho. */
    public static final int BR_STR = 5;

    /** CAN ID del encoder absoluto del módulo delantero izquierdo. */
    public static final int FL_ENC = 12;
    /** CAN ID del encoder absoluto del módulo delantero derecho. */
    public static final int FR_ENC = 11;
    /** CAN ID del encoder absoluto del módulo trasero izquierdo. */
    public static final int BL_ENC = 10;
    /** CAN ID del encoder absoluto del módulo trasero derecho. */
    public static final int BR_ENC = 9;

    /** CAN ID del gyro Pigeon2 usado como IMU principal. */
    public static final int PIGEON = 13;

    // --------------------------------------------------------------------
    // GEOMETRÍA Y CONVERSIONES
    // --------------------------------------------------------------------

    /** Diámetro de la rueda en metros. */
    public static final double WHEEL_DIAMETER = 0.102;

    /**
     * Relación de transmisión del motor de tracción (drive).
     * <p>
     * Vueltas de motor por cada vuelta de rueda.
     */
    public static final double PWR_RATIO = 8.14;

    /**
     * Relación de transmisión del motor de giro (steer).
     * <p>
     * Vueltas de motor por cada vuelta completa del módulo.
     */
    public static final double STR_RATIO = 12.8;

    /**
     * Factor de conversión de rotaciones de motor de tracción a metros
     * recorridos por el módulo.
     *
     * rotaciones_motor * ROT_2_M = metros
     */
    public static final double ROT_2_M =
            (Math.PI * WHEEL_DIAMETER) / PWR_RATIO;

    /**
     * Factor de conversión de rotaciones del motor de giro a radianes de ángulo
     * del módulo.
     *
     * rotaciones_motor * ROT_2_RAD = radianes
     */
    public static final double ROT_2_RAD =
            (2.0 * Math.PI) / STR_RATIO;

    // --------------------------------------------------------------------
    // MOTION MAGIC - DRIVE (VELOCIDAD)
    // --------------------------------------------------------------------

    /** Aceleración de Motion Magic para el motor de tracción (rot/s²). */
    public static final double MAGIC_MOTION_ACC = 400.0;

    /** Jerk de Motion Magic para el motor de tracción (rot/s³). */
    public static final double MAGIC_MOTION_JERK = 4000.0;

    // --------------------------------------------------------------------
    // MOTION MAGIC EXPO - STEER (POSICIÓN)
    // --------------------------------------------------------------------

    /** Velocidad de crucero de Motion Magic para el steer (rot/s). */
    public static final double MAGIC_MOTION_VELOCITY_STR = 95.0;

    /** Aceleración de Motion Magic para el steer (rot/s²). */
    public static final double MAGIC_MOTION_ACCELERATION_STR = 160.0;

    /** Jerk de Motion Magic para el steer (rot/s³). */
    public static final double MAGIC_MOTION_JERK_STR = 160.0 * 10.0;

    /**
     * Ganancia kV del modo Motion Magic Expo para el steer.
     * <p>
     * Escala la contribución de la velocidad en el perfil de movimiento.
     */
    public static final double MAGIC_MOTION_EXPO_KV_STR = 0.12;

    /**
     * Ganancia kA del modo Motion Magic Expo para el steer.
     * <p>
     * Escala la contribución de la aceleración en el perfil de movimiento.
     */
    public static final double MAGIC_MOTION_EXPO_KA_STR = 0.10;

    // --------------------------------------------------------------------
    // GANANCIAS DE CONTROL - POSICIÓN (STEER)
    // --------------------------------------------------------------------

    /**
     * kG: salida para compensar gravedad (en este caso, torque/rozamiento
     * del módulo).
     */
    public static final double POS_KG = 0.20;

    /** kS: salida para vencer fricción estática (offset inicial). */
    public static final double POS_KS = 0.25;

    /** kV: salida por unidad de velocidad objetivo (output / rps). */
    public static final double POS_KV = 0.12;

    /** kA: salida por unidad de aceleración objetivo (output / (rps/s)). */
    public static final double POS_KA = 0.01;

    /** kP: salida por unidad de error de posición (output / rotación). */
    public static final double POS_KP = 4.8;

    /** kI: salida por unidad de error integrado de posición. */
    public static final double POS_KI = 0.0;

    /** kD: salida por unidad de error de velocidad (derivada). */
    public static final double POS_KD = 0.10;

    // --------------------------------------------------------------------
    // GANANCIAS DE CONTROL - VELOCIDAD (DRIVE)
    // --------------------------------------------------------------------

    /** kS: salida para vencer fricción estática en el drive. */
    public static final double VEL_KS = 0.10442;

    /** kV: salida por unidad de velocidad objetivo (output / rps). */
    public static final double VEL_KV = 0.10882;

    /** kA: salida por unidad de aceleración objetivo (output / (rps/s)). */
    public static final double VEL_KA = 0.001647;

    /** kP: salida por unidad de error de velocidad (output / rps). */
    public static final double VEL_KP = 0.34;

    /** kI: salida por unidad de error integrado de velocidad. */
    public static final double VEL_KI = 0.00;

    /** kD: salida por unidad de derivada del error de velocidad. */
    public static final double VEL_KD = 0.003;

    // --------------------------------------------------------------------
    // LIMITES DE ACELERACIÓN / ESTABILIDAD
    // --------------------------------------------------------------------

    /** Aceleración máxima hacia adelante (m/s²) usada en el limitador. */
    public static final double MAX_FORDWARD_ACCEL = 10.0;

    /** Aceleración máxima frontal (m/s²) en el modelo de estabilidad. */
    public static final double MAX_FRONT_ACCEL = 10.0;

    /** Aceleración máxima lateral (m/s²) en el modelo de estabilidad. */
    public static final double MAX_SIDE_ACCEL = 10.0;

    /**
     * Coeficiente de fricción efectivo rueda-suelo.
     * <p>
     * Se usa solo para derivar la aceleración lateral máxima por skid.
     */
    private static final double FRICTION_COF = 0.95;

    /**
     * Aceleración máxima antes de patinar (skid) en m/s².
     * <p>
     * Aproximada como μ * g.
     */
    public static final double MAX_SKID_ACCEL = FRICTION_COF * 9.81;

    /**
     * Zona muerta de velocidad del módulo (m/s).
     * <p>
     * Si la velocidad deseada está dentro de este rango alrededor de 0,
     * se fuerza a 0 para evitar vibraciones.
     */
    public static final double VELOCITY_DEADZONE = 0.10;
}
