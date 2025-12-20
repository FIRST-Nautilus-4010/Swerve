package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.PoseTracker;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Gestor simple de subsistemas/estados del robot.
 *
 * En esta versión reducida, solo se mantiene:
 * <ul>
 *   <li>El subsistema swerve (a través de {@link PoseTracker})</li>
 *   <li>El estado {@link RobotState#TRAVEL}</li>
 * </ul>
 *
 * El objetivo es centrarse únicamente en mover el chasis.
 */
public final class SubsystemManager {

    /** Rastreador de pose del robot (odometría + visión). */
    private final PoseTracker poseTracker;

    /** Estado actual del robot. Solo se usa TRAVEL en esta versión. */
    private RobotState robotState = RobotState.TRAVEL;

    /**
     * Crea el gestor de subsistemas usando el subsistema swerve.
     *
     * @param swerve subsistema de chasis swerve del robot
     */
    public SubsystemManager(Swerve swerve) {
        this.poseTracker = new PoseTracker(swerve);

        // Estado inicial: TRAVEL (conducción normal del chasis).
        scheduleState(RobotState.TRAVEL);
    }

    /**
     * Cambia el estado actual sin lanzar comandos adicionales.
     *
     * @param state nuevo estado del robot
     */
    private void setState(RobotState state) {
        robotState = state;
    }

    /**
     * Ejecuta un cambio explícito de estado:
     * <ul>
     *   <li>Cancela todos los comandos actuales</li>
     *   <li>Programa el nuevo estado</li>
     * </ul>
     *
     * En esta versión, solo existe un comportamiento para TRAVEL.
     */
    public void executeState(RobotState state) {
        CommandScheduler.getInstance().cancelAll();
        scheduleState(state);
    }

    /**
     * Programa el comportamiento asociado a un estado.
     * <p>
     * Actualmente, solo se maneja {@link RobotState#TRAVEL} y el resto
     * de estados se redirigen a TRAVEL.
     */
    public void scheduleState(RobotState state) {
        switch (state) {
            case TRAVEL:
            default:
                // En esta versión, TRAVEL solo actualiza el estado lógico.
                setState(RobotState.TRAVEL);
                break;
        }
    }

    /**
     * Debe llamarse periódicamente desde {@code Robot.periodic()}.
     *
     * Actualiza la estimación de pose y publica el estado actual a
     * SmartDashboard.
     */
    public void periodic() {
        poseTracker.periodic();
        SmartDashboard.putString("Robot State", robotState.toString());
    }
}