package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utilidad para enviar información de debug de un módulo swerve a SmartDashboard.
 * 
 * Centraliza el formato de las keys y el rate-limit de publicación.
 */
public final class SwerveDebugUtil {

    private SwerveDebugUtil() {
        // Clase de utilidades: no instanciable
    }

    /**
     * Publica datos de debug para un módulo swerve, respetando un intervalo mínimo
     * de actualización.
     *
     * @param moduleId           identificador del módulo (por ejemplo, CAN ID del drive)
     * @param desiredFinalVel    velocidad final deseada (m/s)
     * @param currentVel         velocidad actual (m/s)
     * @param nextWantedVel      próxima velocidad objetivo (m/s)
     * @param wantedAcc          aceleración deseada (m/s²)
     * @param limitedAcc         aceleración limitada (m/s²)
     * @param wantedDirection    dirección deseada (rad)
     * @param limitedDirection   dirección limitada (rad)
     * @param lastDebugTime      último tiempo de publicación (segundos FPGA)
     * @param minUpdateInterval  intervalo mínimo entre actualizaciones (segundos)
     * @return nuevo tiempo de última publicación (puede ser el mismo si no publicó)
     */
    public static double publishModuleDebug(
            int moduleId,
            double desiredFinalVel,
            double currentVel,
            double nextWantedVel,
            double wantedAcc,
            double limitedAcc,
            double wantedDirection,
            double limitedDirection,
            double lastDebugTime,
            double minUpdateInterval
    ) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastDebugTime < minUpdateInterval) {
            // No ha pasado suficiente tiempo: no publicamos nada
            return lastDebugTime;
        }

        // Actualizamos el "timestamp" de última publicación
        lastDebugTime = currentTime;

        // Aceleraciones en ejes lateral/frontal, deseadas y limitadas
        SmartDashboard.putNumber("Wanted Acc " + moduleId, wantedAcc);
        SmartDashboard.putNumber("Limited Acc " + moduleId, limitedAcc);

        SmartDashboard.putNumber("Wanted Side Acc " + moduleId,
                wantedAcc * Math.cos(wantedDirection));
        SmartDashboard.putNumber("Wanted Front Acc " + moduleId,
                wantedAcc * Math.sin(wantedDirection));

        SmartDashboard.putNumber("Limited Side Acc " + moduleId,
                limitedAcc * Math.cos(limitedDirection));
        SmartDashboard.putNumber("Limited Front Acc " + moduleId,
                limitedAcc * Math.sin(limitedDirection));

        // Direcciones
        SmartDashboard.putNumber("Wanted Direction " + moduleId, wantedDirection);
        SmartDashboard.putNumber("Limited Direction " + moduleId, limitedDirection);

        // Velocidades
        SmartDashboard.putNumber("Desired Final Vel " + moduleId, desiredFinalVel);
        SmartDashboard.putNumber("Current Vel " + moduleId, currentVel);
        SmartDashboard.putNumber("Next Wanted Vel " + moduleId, nextWantedVel);

        return lastDebugTime;
    }
}