package frc.robot.subsystems.swerve.commands;

// Imports from Java
import java.util.function.Supplier;

// Imports from WPILib
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Constants.ChassisConstants;

/**
 * Comando de conducción manual de un swerve mediante joystick.
 *
 * Toma entradas normalizadas de joystick ([-1, 1]) y las escala a:
 * <ul>
 *   <li>vX, vY en m/s usando {@link ChassisConstants#MAX_VELOCITY}</li>
 *   <li>ω en rad/s usando {@link ChassisConstants#MAX_ANG_SPD}</li>
 * </ul>
 */
public class SwerveDriveJoystick extends Command {

    /** Deadzone de joystick en unidades de velocidad (m/s). */
    private static final double JOYSTICK_DEADZONE =
            0.07 * ChassisConstants.MAX_VELOCITY;

    /** Subsistema swerve controlado por este comando. */
    private final Swerve swerve;

    /** Proveedores de entradas del joystick (normalizadas [-1,1]). */
    private final Supplier<Double> xInput;
    private final Supplier<Double> yInput;
    private final Supplier<Double> zInput;

    /** Modo campo-relativo y petición de reseteo de yaw. */
    private final Supplier<Boolean> fieldRelative;
    private final Supplier<Boolean> resetYaw;

    /**
     * Crea un comando de conducción swerve con joystick.
     *
     * @param swerve        subsistema swerve
     * @param x             eje X del joystick (adelante/atrás, [-1,1])
     * @param y             eje Y del joystick (izquierda/derecha, [-1,1])
     * @param z             eje rotación del joystick (giro, [-1,1])
     * @param fieldRelative si {@code true}, conducción relativa al campo
     * @param resetYaw      si {@code true}, resetea el heading del gyro
     */
    public SwerveDriveJoystick(
            Swerve swerve,
            Supplier<Double> x,
            Supplier<Double> y,
            Supplier<Double> z,
            Supplier<Boolean> fieldRelative,
            Supplier<Boolean> resetYaw
    ) {
        this.swerve = swerve;
        this.xInput = x;
        this.yInput = y;
        this.zInput = z;
        this.fieldRelative = fieldRelative;
        this.resetYaw = resetYaw;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Lee entradas normalizadas y las escala a velocidades físicas.
        double xSpeed = xInput.get() * ChassisConstants.MAX_VELOCITY;  // m/s
        double ySpeed = yInput.get() * ChassisConstants.MAX_VELOCITY;  // m/s
        double zSpeed = zInput.get() * ChassisConstants.MAX_ANG_SPD;   // rad/s

        // Aplica deadzone para evitar movimientos muy pequeños no deseados.
        xSpeed = applyDeadzone(xSpeed, JOYSTICK_DEADZONE);
        ySpeed = applyDeadzone(ySpeed, JOYSTICK_DEADZONE);
        zSpeed = applyDeadzone(zSpeed, JOYSTICK_DEADZONE);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);

        if (fieldRelative.get()) {
            // Conducción relativa al campo (usa orientación actual del robot).
            swerve.driveFieldRelative(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    chassisSpeeds.omegaRadiansPerSecond
            );
        } else {
            // Conducción relativa al robot.
            swerve.drive(chassisSpeeds);
        }

        // Opción para resetear yaw del gyro (por ejemplo, botón en el joystick).
        if (resetYaw.get()) {
            swerve.zeroHeading();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Detiene todos los módulos cuando el comando termina o es interrumpido.
        swerve.stopModules();
    }

    /** Aplica una deadzone simétrica alrededor de 0. */
    private static double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }
}