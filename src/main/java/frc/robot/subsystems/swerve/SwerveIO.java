package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Gestiona el hardware de un solo módulo swerve:
 * <ul>
 *   <li>Motor de tracción (drive)</li>
 *   <li>Motor de giro (steer/turning)</li>
 *   <li>Encoder absoluto del ángulo</li>
 * </ul>
 *
 * Esta clase se encarga de:
 * <ul>
 *   <li>Inicializar los dispositivos CAN</li>
 *   <li>Sincronizar el encoder relativo del motor de giro con el encoder absoluto</li>
 *   <li>Proveer métodos de acceso a posición y velocidad del motor de tracción</li>
 * </ul>
 */
public class SwerveIO {

    /** Motor responsable de mover la rueda hacia adelante/atrás. */
    private final TalonFX driveMotor;

    /** Motor responsable de girar la rueda (cambiar orientación del módulo). */
    private final TalonFX turningMotor;

    /** Encoder absoluto que mide el ángulo real del módulo swerve. */
    private final CANcoder absoluteEncoder;

    /**
     * Offset del encoder absoluto en radianes.
     * <p>
     * Sirve para alinear mecánicamente el 0 del módulo con el 0 de software.
     * Si no tienes offset aún, puedes dejarlo en 0 y calibrarlo luego.
     */
    private final double absoluteEncoderOffsetRad;

    /**
     * Crea un nuevo SwerveIO para un módulo swerve.
     *
     * @param driveTalonFxId       ID CAN del TalonFX de tracción
     * @param turningTalonFxId     ID CAN del TalonFX de giro
     * @param absoluteEncoderId    ID CAN del encoder absoluto (CANcoder)
     * @param absoluteOffsetRad    offset del encoder absoluto en radianes
     *                             (ángulo mecánico cuando consideras que el módulo está a 0)
     */
    public SwerveIO(
            int driveTalonFxId,
            int turningTalonFxId,
            int absoluteEncoderId,
            double absoluteOffsetRad
    ) {
        this.absoluteEncoder = new CANcoder(absoluteEncoderId, "cleopatra");
        this.driveMotor = new TalonFX(driveTalonFxId, "cleopatra");
        this.turningMotor = new TalonFX(turningTalonFxId, "cleopatra");
        this.absoluteEncoderOffsetRad = absoluteOffsetRad;

        // Sincroniza el encoder relativo del motor de giro con el encoder absoluto
        // y pone a cero la posición del motor de tracción.
        resetEncoders();
    }

    /**
     * Restaura y sincroniza los encoders del módulo.
     * <ul>
     *   <li>El motor de giro toma como referencia el valor del encoder absoluto,
     *       convertido a "vueltas de módulo" según la relación de engranajes.</li>
     *   <li>El motor de tracción se pone a 0 metros recorridos.</li>
     * </ul>
     */
    public final void resetEncoders() {
        // Convierte el ángulo absoluto (rad) a vueltas del módulo,
        // luego aplica la relación de transmisión del mecanismo de giro.
        double turningMotorRotations =
                (getAbsoluteEncoderRadians() / (2.0 * Math.PI)) * SwerveConstants.STR_RATIO;

        turningMotor.setPosition(turningMotorRotations);

        // El encoder del drive se reinicia a 0 (punto de referencia de distancia).
        driveMotor.setPosition(0.0);
    }

    /**
     * Detiene ambos motores del módulo swerve.
     * <p>
     * Útil cuando quieres cortar inmediatamente cualquier movimiento del módulo.
     */
    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    /**
     * Devuelve el ángulo del encoder absoluto en radianes, ajustado con el offset.
     *
     * @return ángulo del módulo en radianes, en el rango [0, 2π) aproximadamente,
     *         corregido por el {@code absoluteEncoderOffsetRad}.
     */
    public double getAbsoluteEncoderRadians() {
        // El CANcoder devuelve normalmente un valor en vueltas [0, 1).
        double rawRotations = absoluteEncoder
                .getAbsolutePosition()
                .getValue()
                .magnitude();

        // Convierte de vueltas a radianes: rotaciones * 2π.
        double angleRad = rawRotations * 2.0 * Math.PI;

        // Aplica offset para alinear el 0 mecánico al 0 lógico.
        angleRad -= absoluteEncoderOffsetRad;

        // Opcional: normalizar al rango [0, 2π)
        angleRad = Math.IEEEremainder(angleRad, 2.0 * Math.PI);
        if (angleRad < 0) {
            angleRad += 2.0 * Math.PI;
        }

        return angleRad;
    }

    /**
     * Devuelve el motor de tracción (drive).
     * <p>
     * Se expone para que otras clases puedan configurar PID, modos de control, etc.
     *
     * @return instancia del {@link TalonFX} de tracción.
     */
    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    /**
     * Devuelve el motor de giro (turning/steer).
     *
     * @return instancia del {@link TalonFX} de giro.
     */
    public TalonFX getTurningMotor() {
        return turningMotor;
    }

    /**
     * Devuelve la velocidad lineal del módulo.
     *
     * @return velocidad del módulo en metros por segundo.
     *         Se obtiene desde la velocidad del TalonFX
     *         y se convierte usando {@link SwerveConstants#ROT_2_M}.
     */
    public double getDriveMotorVelocityMetersPerSecond() {
        double motorVelocityRotationsPerUnit =
                driveMotor.getVelocity().getValueAsDouble();

        // ROT_2_M: factor que convierte rotaciones del motor a metros recorridos.
        return motorVelocityRotationsPerUnit * SwerveConstants.ROT_2_M;
    }

    /**
     * Devuelve la distancia recorrida por el módulo.
     *
     * @return posición del drive en metros, desde el último {@link #resetEncoders()}.
     */
    public double getDriveMotorPositionMeters() {
        double motorPositionRotations =
                driveMotor.getPosition().getValueAsDouble();

        // Convierte rotaciones acumuladas a metros totales recorridos.
        return motorPositionRotations * SwerveConstants.ROT_2_M;
    }
}