// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.SwerveDriveJoystick;

/**
 * Clase central de configuración del robot.
 *
 * Para este proyecto, solo se configura lo necesario para mover el chasis
 * swerve con un control Xbox.
 */
public class RobotContainer {

  /** Subsistema swerve principal del robot. */
  private final Swerve swerve;

  /** Joystick del driver (puerto 0). */
  private final XboxController driverJoystick;

  /**
   * Crea el contenedor del robot.
   *
   * Inicializa el subsistema swerve y configura el comando por defecto
   * de conducción con joystick.
   */
  public RobotContainer() {
    // true = usar Pigeon2 como IMU principal, false = usar NavX.
    this.swerve = new Swerve(true);

    this.driverJoystick = new XboxController(0);

    configureBindings();
  }

  /**
   * Configura los bindings de comandos:
   * <ul>
   *   <li>Comando por defecto del swerve: conducción con joystick</li>
   * </ul>
   */
  private void configureBindings() {
    // Comando por defecto: controlar el swerve con el joystick del driver.
    swerve.setDefaultCommand(
        new SwerveDriveJoystick(
            swerve,
            // Eje Y del joystick izquierdo controla movimiento hacia adelante/atrás.
            () -> -driverJoystick.getLeftY(),
            // Eje X del joystick izquierdo controla movimiento lateral.
            () -> -driverJoystick.getLeftX(),
            // Eje X del joystick derecho controla la rotación.
            () -> -driverJoystick.getRightX(),
            // Botón X: cuando NO está presionado -> modo field-relative (true).
            () -> !driverJoystick.getXButton(),
            // Botón A: resetea el yaw del gyro.
            () -> driverJoystick.getAButton()
        ));
  }

  /**
   * Comando autónomo por defecto.
   * <p>
   * De momento no se ha configurado un autónomo real, así que devuelve
   * un {@link InstantCommand} vacío.
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
