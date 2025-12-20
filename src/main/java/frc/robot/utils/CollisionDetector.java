package frc.robot.utils;

import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Detector sencillo de colisiones/impactos basándose en la aceleración lineal
 * medida por el subsistema {@link Swerve}.
 *
 * <p>Cuando la aceleración supera un umbral, se considera que ha habido un
 * impacto y se activa un periodo de "congelado" durante varios ciclos,
 * durante el cual otros sistemas (por ejemplo, la odometría) pueden decidir
 * no confiar en los datos.</p>
 */
public class CollisionDetector {

    /**
     * Umbral de aceleración lineal (m/s²) a partir del cual se considera impacto.
     * <p>
     * Se toma el valor máximo esperado de aceleración del chasis y se le suma
     * un margen de seguridad.
     */
    private static final double IMPACT_THRESHOLD =
            ChassisConstants.MAX_ACCEL + 1.0;

    /** Indica si se ha detectado un impacto recientemente. */
    private boolean recentlyHit = false;

    /**
     * Contador de ciclos restantes durante los cuales se mantiene el estado
     * de "congelado" tras un impacto.
     */
    private int freezeCounter = 0;

    /**
     * Analiza la aceleración actual del robot y detecta si se ha producido
     * un impacto fuerte.
     *
     * @param swerve subsistema swerve, del que se obtiene la aceleración lineal
     * @return {@code true} si estamos en estado de impacto reciente, {@code false} en caso contrario
     */
    public boolean detectImpact(Swerve swerve) {
        // Aceleración lineal total (m/s²), implementada en Swerve.getLinearAcceleration().
        double accel = swerve.getLinearAcceleration();

        if (Math.abs(accel) > IMPACT_THRESHOLD) {
            recentlyHit = true;
            // Congela durante 5 ciclos (~100 ms a 20 ms por ciclo).
            freezeCounter = 5;
        }

        return recentlyHit;
    }

    /**
     * Avanza el estado del "congelado" tras una colisión.
     * <p>
     * Debería llamarse periódicamente cuando se ha detectado impacto y se
     * quiere esperar unos ciclos antes de volver a confiar en la odometría.
     */
    public void freeze() {
        if (freezeCounter > 0) {
            freezeCounter--;
        } else {
            recentlyHit = false;
        }
    }
}
