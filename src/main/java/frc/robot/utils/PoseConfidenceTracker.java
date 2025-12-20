package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Rastrea la "confianza" en la odometría del robot.
 *
 * <p>Se basa en:</p>
 * <ul>
 *   <li>Diferencia entre velocidad media de ruedas y velocidad del chasis
 *       para detectar skid (patinaje).</li>
 *   <li>Diferencia entre pose de visión y odometría para decidir si
 *       confiar en la medición de cámara.</li>
 * </ul>
 */
public class PoseConfidenceTracker {

    /** Indica si el robot está probablemente patinando (skid). */
    private boolean skidding = false;

    /**
     * Umbral de diferencia entre velocidad media de rueda y velocidad
     * del chasis a partir del cual se considera que hay skid (m/s).
     */
    private static final double SKID_SPEED_DIFF_THRESHOLD = 0.5;

    /**
     * Umbral de distancia máxima entre visión y odometría para confiar
     * en la medición de visión (metros).
     */
    private static final double VISION_TRUST_DISTANCE_THRESHOLD_METERS = 1.0;

    /**
     * Actualiza el estado interno de skid en base al subsistema swerve.
     *
     * @param swerve subsistema swerve del que se leen velocidades
     */
    public void update(Swerve swerve) {
        double avgWheelSpeed = swerve.getAverageWheelSpeed();
        double chassisSpeed = swerve.getChassisSpeed();

        // Si las ruedas "dicen" ir mucho más rápido que el chasis, probablemente hay skid.
        skidding = Math.abs(avgWheelSpeed - chassisSpeed) > SKID_SPEED_DIFF_THRESHOLD;
    }

    /** @return {@code true} si se detecta posible skid (poca confianza en odometría). */
    public boolean isSkidding() {
        return skidding;
    }

    /**
     * Decide si se debe confiar en una medición de visión comparándola
     * con la pose de odometría actual.
     *
     * @param vision pose medida por visión (por ejemplo, Limelight / AprilTags)
     * @param odom   pose estimada por odometría
     * @return {@code true} si la diferencia entre ambas es menor que el umbral
     */
    public boolean shouldTrustVision(Pose2d vision, Pose2d odom) {
        double distance = vision.getTranslation().getDistance(odom.getTranslation());
        return distance < VISION_TRUST_DISTANCE_THRESHOLD_METERS;
    }
}
