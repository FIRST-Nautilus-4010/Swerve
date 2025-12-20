package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ChassisConstants;

/**
 * Subsistema principal de swerve del robot.
 *
 * Se encarga de:
 * <ul>
 *   <li>Gestionar los 4 módulos swerve</li>
 *   <li>Leer orientación y aceleraciones de gyro (Pigeon2 o NavX)</li>
 *   <li>Convertir velocidades de chasis en estados de módulos</li>
 *   <li>Publicar estados actuales y deseados a NetworkTables</li>
 * </ul>
 */
public class Swerve extends SubsystemBase {

    // --------------------------------------------------------------------
    // MÓDULOS
    // --------------------------------------------------------------------

    /** Módulo delantero izquierdo. */
    private final SwerveModule frontLeft =
            new SwerveModule(SwerveConstants.FL_PWR, SwerveConstants.FL_STR, SwerveConstants.FL_ENC);

    /** Módulo delantero derecho. */
    private final SwerveModule frontRight =
            new SwerveModule(SwerveConstants.FR_PWR, SwerveConstants.FR_STR, SwerveConstants.FR_ENC);

    /** Módulo trasero izquierdo. */
    private final SwerveModule backLeft =
            new SwerveModule(SwerveConstants.BL_PWR, SwerveConstants.BL_STR, SwerveConstants.BL_ENC);

    /** Módulo trasero derecho. */
    private final SwerveModule backRight =
            new SwerveModule(SwerveConstants.BR_PWR, SwerveConstants.BR_STR, SwerveConstants.BR_ENC);

    // --------------------------------------------------------------------
    // SENSORES DE ORIENTACIÓN
    // --------------------------------------------------------------------

    /** NavX (backup o alternativa al Pigeon). */
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    /** Pigeon2 como IMU principal. */
    private final Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON, "cleopatra");

    /**
     * Indica si se debe usar el Pigeon2 como fuente principal de orientación.
     * Si es {@code false}, se usa el NavX.
     */
    private boolean usePigeon = true;

    // --------------------------------------------------------------------
    // PUBLICADORES A NETWORKTABLES
    // --------------------------------------------------------------------

    /** Estados medidos de los módulos (velocidad + ángulo). */
    private final StructArrayPublisher<SwerveModuleState> swervePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Detected module states", SwerveModuleState.struct)
                    .publish();

    /** Estados deseados de los módulos (comando). */
    private final StructArrayPublisher<SwerveModuleState> swerveDesiredStatePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("desiredStates", SwerveModuleState.struct)
                    .publish();

    /**
     * Crea el subsistema Swerve.
     *
     * @param usePigeon si {@code true}, se usa Pigeon2; si {@code false}, NavX.
     */
    public Swerve(boolean usePigeon) {
        this.usePigeon = usePigeon;

        // Resetea heading al inicializar el subsistema.
        zeroHeading();
    }

    // --------------------------------------------------------------------
    // CICLO PERIÓDICO
    // --------------------------------------------------------------------

    @Override
    public void periodic() {
        // Publica estados actuales de los módulos a NetworkTables.
        swervePublisher.set(getSwerveModuleStates());

        // Telemetría básica a SmartDashboard.
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    // --------------------------------------------------------------------
    // ESTADOS DE MÓDULO / CHASIS
    // --------------------------------------------------------------------

    /** Devuelve las posiciones actuales de los 4 módulos (para odometría). */
    public SwerveModulePosition[] getSwerveModulePos() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    /** Devuelve los estados actuales de los 4 módulos (velocidad + ángulo). */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    /** Heading actual del robot en grados. */
    public double getHeading() {
        if (usePigeon) {
            return pigeon.getYaw().getValueAsDouble();
        } else {
            // NavX usa convención opuesta, por eso el signo negativo.
            return -gyro.getAngle();
        }
    }

    /** Pitch actual del robot en grados. */
    public double getPitch() {
        if (usePigeon) {
            return pigeon.getPitch().getValueAsDouble();
        } else {
            return gyro.getPitch();
        }
    }

    /** Roll actual del robot en grados. */
    public double getRoll() {
        if (usePigeon) {
            return pigeon.getRoll().getValueAsDouble();
        } else {
            return gyro.getRoll();
        }
    }

    /** Devuelve la orientación actual como {@link Rotation2d}. */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // --------------------------------------------------------------------
    // ACELERACIONES
    // --------------------------------------------------------------------

    /** Aceleración lineal en X (m/s²) en el marco del robot. */
    public double getAccelX() {
        if (usePigeon) {
            return pigeon.getAccelerationX().getValue().magnitude();
        } else {
            return gyro.getWorldLinearAccelX();
        }
    }

    /** Aceleración lineal en Y (m/s²) en el marco del robot. */
    public double getAccelY() {
        if (usePigeon) {
            return pigeon.getAccelerationY().getValue().magnitude();
        } else {
            return gyro.getWorldLinearAccelY();
        }
    }

    /** Aceleración lineal en Z (m/s²). */
    public double getAccelZ() {
        if (usePigeon) {
            return pigeon.getAccelerationZ().getValue().magnitude();
        } else {
            return gyro.getWorldLinearAccelZ();
        }
    }

    /** Módulo de la aceleración lineal total. */
    public double getLinearAcceleration() {
        double ax = getAccelX();
        double ay = getAccelY();
        double az = getAccelZ();
        return Math.sqrt(ax * ax + ay * ay + az * az);
    }

    /** Velocidad media de las ruedas (m/s). */
    public double getAverageWheelSpeed() {
        SwerveModuleState[] states = getSwerveModuleStates();
        double sum = 0.0;
        for (SwerveModuleState state : states) {
            sum += state.speedMetersPerSecond;
        }
        return sum / states.length;
    }

    /** Velocidad lineal del chasis calculada a partir de los estados de módulos. */
    public double getChassisSpeed() {
        ChassisSpeeds speeds =
                ChassisConstants.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());

        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    // --------------------------------------------------------------------
    // GYRO
    // --------------------------------------------------------------------

    /** Resetea el heading del sensor de orientación actual. */
    public void zeroHeading() {
        if (usePigeon) {
            pigeon.reset();
        } else {
            gyro.reset();
        }
    }

    // --------------------------------------------------------------------
    // CONTROL DE MÓDULOS
    // --------------------------------------------------------------------

    /** Detiene los 4 módulos swerve. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Conduce el robot en modo field-relative.
     *
     * @param xSpeed velocidad en X relativa al campo (m/s)
     * @param ySpeed velocidad en Y relativa al campo (m/s)
     * @param rot    velocidad angular (rad/s)
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
        // Convierte velocidades del marco del campo al marco del robot.
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        rot,
                        getRotation2d());

        drive(fieldRelativeSpeeds);
    }

    /**
     * Conduce el robot con velocidades en el marco del robot.
     *
     * @param speeds velocidades de chasis (vx, vy, ω)
     */
    public void drive(ChassisSpeeds speeds) {
        // Convierte velocidades de chasis a estados de módulos.
        SwerveModuleState[] moduleStates =
                ChassisConstants.KINEMATICS.toSwerveModuleStates(speeds);

        // Aplica estados a los módulos.
        setStates(moduleStates);

        // Publica estados deseados a NetworkTables (telemetría).
        swerveDesiredStatePublisher.set(moduleStates);
    }

    /**
     * Aplica estados deseados a cada módulo, desaturando si es necesario.
     *
     * @param desiredStates array de 4 estados de módulo en el orden:
     *                      FL, FR, BL, BR.
     */
    public void setStates(SwerveModuleState[] desiredStates) {
        // Asegura que ninguna rueda exceda la velocidad máxima.
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                ChassisConstants.MAX_VELOCITY);

        double roll = getRoll();
        double pitch = getPitch();

        frontLeft.setDesiredState(desiredStates[0], roll, pitch);
        frontRight.setDesiredState(desiredStates[1], roll, pitch);
        backLeft.setDesiredState(desiredStates[2], roll, pitch);
        backRight.setDesiredState(desiredStates[3], roll, pitch);
    }
}
