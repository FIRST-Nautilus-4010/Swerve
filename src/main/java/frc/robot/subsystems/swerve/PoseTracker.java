package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.swerve.commands.DriveTo;
import frc.robot.subsystems.swerve.commands.SwerveDriveJoystick;
import frc.robot.utils.PoseConfidenceTracker;
import frc.robot.utils.CollisionDetector;
import frc.robot.utils.LimelightHelpers;

/**
 * Encapsula toda la lógica de estimación de pose del robot.
 *
 * Fuentes de información:
 * <ul>
 *   <li>Odometry de los módulos swerve + gyro</li>
 *   <li>Medidas de visión (Limelight / AprilTags)</li>
 *   <li>Detección de colisión y skid para ajustar la confianza</li>
 * </ul>
 *
 * También expone comandos de alto nivel como {@link #driveTo(Pose2d)}.
 */
public class PoseTracker {

    // --- Constantes internas ---

    /** Nombre de la cámara Limelight usada para odometría. */
    private static final String LIMELIGHT_NAME = "limelight";

    /** Umbral mínimo de área de target para considerar la medición válida. */
    private static final double MIN_TARGET_AREA = 0.1;

    // --- Estimador de pose y publicación ---

    /** Estimador de pose basado en modelo swerve + mediciones de visión. */
    private final SwerveDrivePoseEstimator poseEstimator;

    /** Publicador de pose al NetworkTables para dashboards externos. */
    private final StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("Robot position", Pose2d.struct)
                    .publish();

    // --- Dependencias del subsistema ---

    /** Referencia al subsistema de swerve, del que se obtienen módulos y gyro. */
    private final Swerve swerve;

    // --- Utilidades auxiliares ---

    /** Rastrea confianza en la odometría (detección de skid, etc.). */
    private final PoseConfidenceTracker confidenceTracker = new PoseConfidenceTracker();

    /** Detecta impactos/colisiones que puedan invalidar la odometría. */
    private final CollisionDetector collisionDetector = new CollisionDetector();

    /** Indica si ya se inicializó la pose usando visión (Limelight). */
    private boolean initialPoseSetFromVision = false;

    /**
     * Crea un {@link PoseTracker} asociado a un subsistema swerve.
     *
     * Inicializa el {@link SwerveDrivePoseEstimator} usando:
     * <ul>
     *   <li>Cinemática del chasis</li>
     *   <li>Rotación inicial (0 rad)</li>
     *   <li>Posiciones iniciales de módulos</li>
     *   <li>Pose inicial definida en {@link AutonomousConstants#initialPose}</li>
     * </ul>
     *
     * @param swerve subsistema swerve del robot.
     */
    public PoseTracker(Swerve swerve) {
        this.swerve = swerve;

        this.poseEstimator = new SwerveDrivePoseEstimator(
                ChassisConstants.KINEMATICS,
                new Rotation2d(0.0),
                swerve.getSwerveModulePos(),
                AutonomousConstants.initialPose
        );
    }

    // --------------------------------------------------------------------
    // API PRINCIPAL
    // --------------------------------------------------------------------

    /** Devuelve la pose estimada actual del robot. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resetea la odometría a una pose dada.
     *
     * @param pose nueva pose de referencia.
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                swerve.getRotation2d(),
                swerve.getSwerveModulePos(),
                pose
        );
    }

    /** Crea un comando para conducir hasta una pose objetivo. */
    public Command driveTo(Pose2d pose) {
        return new DriveTo(pose, swerve, this);
    }

    /**
     * Crea un comando para rotar hasta un ángulo objetivo
     * manteniendo la posición X/Y actual.
     */
    public Command rotateTo(Rotation2d angle) {
        Pose2d current = getPose();
        return new DriveTo(new Pose2d(current.getX(), current.getY(), angle), swerve, this);
    }

    /**
     * Crea un comando de conducción manual con joystick.
     *
     * @param vx            velocidad X (m/s)
     * @param vy            velocidad Y (m/s)
     * @param omega         velocidad angular (rad/s)
     * @param fieldRelative si es relativo al campo
     * @param resetYaw      si se quiere resetear yaw
     */
    public Command setSpeeds(
            Supplier<Double> vx,
            Supplier<Double> vy,
            Supplier<Double> omega,
            Supplier<Boolean> fieldRelative,
            Supplier<Boolean> resetYaw
    ) {
        return new SwerveDriveJoystick(swerve, vx, vy, omega, fieldRelative, resetYaw);
    }

    // --------------------------------------------------------------------
    // VISIÓN (LIMELIGHT)
    // --------------------------------------------------------------------

    /**
     * Devuelve la pose estimada por la Limelight si hay un target válido.
     *
     * @return {@link Optional} con la pose de visión en coordenadas WPI Blue,
     *         o vacío si no hay medición confiable.
     */
    private Optional<Pose2d> getVisionPose() {
        boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
        double targetArea = LimelightHelpers.getTA(LIMELIGHT_NAME);

        if (hasTarget && targetArea > MIN_TARGET_AREA) {
            Pose2d botPose =
                    LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);
            return Optional.of(botPose);
        }

        return Optional.empty();
    }

    /**
     * Timestamp (segundos FPGA) correspondiente a la última medición de visión.
     *
     * Se obtiene restando la latencia de captura que entrega Limelight a
     * {@link Timer#getFPGATimestamp()}.
     */
    private double getLastVisionTimestamp() {
        double captureLatencyMs = LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME);
        return Timer.getFPGATimestamp() - captureLatencyMs / 1000.0;
    }

    // --------------------------------------------------------------------
    // CICLO PERIÓDICO
    // --------------------------------------------------------------------

    /**
     * Debe ser llamado periódicamente (por ejemplo, desde {@code Swerve.periodic()}).
     *
     * Flujo general:
     * <ol>
     *   <li>Detectar colisiones y decidir si actualizar solo por odometría.</li>
     *   <li>Actualizar confianza (skid) y ajustar std dev de visión.</li>
     *   <li>Procesar mediciones de Limelight (inicialización + fusión).</li>
     *   <li>Publicar la pose a SmartDashboard y NetworkTables.</li>
     * </ol>
     */
    public void periodic() {
        // ======= 1. Actualización por odometría / colisión =======
        if (collisionDetector.detectImpact(swerve)) {
            // Se ha detectado una colisión fuerte: congela temporalmente odometría.
            collisionDetector.freeze();
        } else {
            // Actualiza pose usando solo gyro + módulos swerve.
            poseEstimator.update(
                    swerve.getRotation2d(),
                    swerve.getSwerveModulePos()
            );
        }

        // ======= 2. Skid detection / confianza en visión =======
        confidenceTracker.update(swerve);

        if (confidenceTracker.isSkidding()) {
            // Cuando el robot patina, confiamos menos en la odometría
            // y por tanto aumentamos la varianza asociada a las mediciones de visión.
            poseEstimator.setVisionMeasurementStdDevs(
                    AutonomousConstants.LOW_CONFIDENCE_STD
            );
        } else {
            poseEstimator.setVisionMeasurementStdDevs(
                    AutonomousConstants.NORMAL_CONFIDENCE_STD
            );
        }

        // ======= 3. Actualizaciones de visión (AprilTags / Limelight) =======
        Optional<Pose2d> visionMeasurement = getVisionPose();

        if (visionMeasurement.isPresent()) {
            Pose2d visionPose = visionMeasurement.get();
            double timestamp = getLastVisionTimestamp();

            // Si aún no hemos fijado la posición inicial con visión,
            // reseteamos completamente la odometría a la pose de la cámara.
            if (!initialPoseSetFromVision) {
                resetOdometry(visionPose);
                initialPoseSetFromVision = true;
                SmartDashboard.putString("Init Pose Source", "Limelight");
            }

            // Solo fusiona visión si el tracker de confianza lo permite.
            if (confidenceTracker.shouldTrustVision(visionPose, getPose())) {
                poseEstimator.addVisionMeasurement(visionPose, timestamp);
            }
        }

        // ======= 4. Publicación a Dashboard / NT =======
        Pose2d estimatedPose = getPose();
        SmartDashboard.putString("Robot Pose", estimatedPose.toString());
        posePublisher.set(estimatedPose);
    }
}
