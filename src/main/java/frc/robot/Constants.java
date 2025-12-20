package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Conjunto de constantes globales del proyecto.
 */
public final class Constants {

  private Constants() {
    // Clase de solo constantes: no instanciable.
  }

  // ------------------------------------------------------------------------
  // CHASSIS / SWERVE
  // ------------------------------------------------------------------------
  public static final class ChassisConstants {

    private ChassisConstants() {}

    /** Distancia entre ruedas derecha e izquierda (m). */
    public static final double TRACKWIDTH = 0.42;

    /** Distancia entre ruedas delanteras y traseras (m). */
    public static final double WHEELBASE = 0.42;

    /**
     * Cinemática del chasis swerve.
     * <p>
     * Define la posición de cada módulo respecto al centro del robot.
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH / 2.0,  WHEELBASE / 2.0),   // Front Left
        new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),   // Front Right
        new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),   // Back Left
        new Translation2d(-TRACKWIDTH / 2.0,-WHEELBASE / 2.0));  // Back Right

    /** Velocidad lineal máxima del chasis (m/s). */
    public static final double MAX_VELOCITY = 3.77952;

    /** Velocidad angular máxima del chasis (rad/s). */
    public static final double MAX_ANG_SPD = 4.79 * Math.PI;

    /** Aceleración lineal máxima esperada del chasis (m/s²). */
    public static final double MAX_ACCEL = 479.0;

    /** Aceleración angular máxima esperada del chasis (rad/s²). */
    public static final double MAX_ANG_ACCEL = 479.0;
  }

  // ------------------------------------------------------------------------
  // AUTÓNOMO / POSE ESTIMATOR
  // ------------------------------------------------------------------------
  public static final class AutonomousConstants {

    private AutonomousConstants() {}

    /** Pose inicial del robot en el campo. */
    public static final Pose2d initialPose = new Pose2d();

    // Ganancias PID para control de posición X.
    public static final double P_X = 50.0;
    public static final double I_X = 0.0;
    public static final double D_X = 0.0;

    // Ganancias PID para control de posición Y.
    public static final double P_Y = 50.0;
    public static final double I_Y = 0.0;
    public static final double D_Y = 0.0;

    // Ganancias PID para control de ángulo (theta).
    public static final double P_Z = 3.0;
    public static final double I_Z = 0.0;
    public static final double D_Z = 0.0;

    /** Velocidad lineal máxima permitida en auton (m/s). */
    public static final double MAX_SPD = ChassisConstants.MAX_VELOCITY;

    /** Aceleración lineal máxima permitida en auton (m/s²). */
    public static final double MAX_ACCEL = ChassisConstants.MAX_ACCEL;

    /** Velocidad angular máxima permitida en auton (rad/s). */
    public static final double MAX_ANG_SPD = ChassisConstants.MAX_ANG_SPD;

    /** Aceleración angular máxima permitida en auton (rad/s²). */
    public static final double MAX_ANG_ACCEL = ChassisConstants.MAX_ANG_ACCEL;

    /**
     * Constraints para el ProfiledPIDController de theta (rotación),
     * usados por el {@link edu.wpi.first.math.controller.HolonomicDriveController}.
     */
    public static final TrapezoidProfile.Constraints Z_CONTROLER =
        new TrapezoidProfile.Constraints(
            MAX_ANG_SPD,
            MAX_ANG_ACCEL);

    /** Tolerancia de posición para finalizar un movimiento auton (m). */
    public static final double POS_TOLERANCE = 0.05;

    /** Tolerancia de ángulo para finalizar un movimiento auton (rad). */
    public static final double ANG_TOLERANCE = Math.toRadians(5.0);

    // --------------------------------------------------------------------
    // COVARIANZAS DE VISIÓN PARA EL POSE ESTIMATOR
    // --------------------------------------------------------------------

    /** Desviación estándar base para X/Y en estado de confianza normal. */
    public static final double NORMAL_STD = 0.003;

    /** Matriz de desviaciones estándar para visión en estado de confianza normal. */
    public static final Matrix<N3, N1> NORMAL_CONFIDENCE_STD;

    static {
      NORMAL_CONFIDENCE_STD = new Matrix<>(N3.instance, N1.instance);
      NORMAL_CONFIDENCE_STD.set(0, 0, NORMAL_STD);              // X
      NORMAL_CONFIDENCE_STD.set(1, 0, NORMAL_STD);              // Y
      NORMAL_CONFIDENCE_STD.set(2, 0, Math.toRadians(3.0));     // Theta
    }

    /** Desviación estándar base para X/Y en estado de baja confianza. */
    public static final double LOW_STD = 0.05;

    /** Matriz de desviaciones estándar para visión en estado de baja confianza. */
    public static final Matrix<N3, N1> LOW_CONFIDENCE_STD;

    static {
      LOW_CONFIDENCE_STD = new Matrix<>(N3.instance, N1.instance);
      LOW_CONFIDENCE_STD.set(0, 0, LOW_STD);                    // X
      LOW_CONFIDENCE_STD.set(1, 0, LOW_STD);                    // Y
      LOW_CONFIDENCE_STD.set(2, 0, Math.toRadians(5.0));        // Theta
    }
  }
}
