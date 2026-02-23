/**
 *******************************************************************************
 * @file    STM32_AHT20_conf.h
 * @author  STM32_LIB_STYLE_GUIDE
 * @brief   Configuration overridable STM32_AHT20 — surcharger via -D ou
 *          en définissant les macros AVANT d'inclure STM32_AHT20.h
 * @version 0.9.0
 * @date    2026-02-22
 * @copyright Libre sous licence MIT.
 *******************************************************************************
 */
#ifndef STM32_AHT20_CONF_H
#define STM32_AHT20_CONF_H

/* ============================================================================
 * Debug / traces série (désactivé par défaut en production)
 * ============================================================================ */

/**
 * @brief Active AHT20_StatusToString() et d'éventuelles traces.
 *
 * Décommenter pour activer — ou définir via option de compilation.
 * En production (firmware final), laisser commenté pour économiser la Flash.
 *
 * @note  AHT20_StatusToString() retourne "" (chaîne vide) si non défini.
 */
/* #define AHT20_DEBUG_ENABLE */

/* ============================================================================
 * Timeouts I2C
 * ============================================================================ */

/**
 * @brief Timeout I2C par défaut en ms
 * @note  Datasheet AHT20 §5.4 : opérations I2C < 1ms typ — 100ms offre une marge confortable.
 */
#ifndef AHT20_DEFAULT_TIMEOUT_MS
#define AHT20_DEFAULT_TIMEOUT_MS   100U
#endif

/**
 * @brief Timeout watchdog d'attente mesure async en ms
 * @note  Datasheet AHT20 §5.4 : conversion ≥75ms typ → 200ms offre ×2.5 de marge
 *        sans bloquer le Watchdog sur un bus très chargé.
 */
#ifndef AHT20_WAIT_TIMEOUT_MS
#define AHT20_WAIT_TIMEOUT_MS      200U
#endif

/* ============================================================================
 * Gestion d'erreurs
 * ============================================================================ */

/**
 * @brief Seuil d'erreurs consécutives avant signalement critique
 * @note  Au-delà de ce seuil, l'application peut décider un SoftReset ou un DeInit/Init.
 */
#ifndef AHT20_MAX_CONSECUTIVE_ERRORS
#define AHT20_MAX_CONSECUTIVE_ERRORS  3U
#endif

/**
 * @brief Intervalle minimal par défaut entre deux mesures async (ms)
 * @note  Surcharger via haht20.sample_interval_ms après AHT20_Init()
 *        ou via -DAHT20_DEFAULT_SAMPLE_INTERVAL_MS=<valeur> dans le compilateur.
 */
#ifndef AHT20_DEFAULT_SAMPLE_INTERVAL_MS
#define AHT20_DEFAULT_SAMPLE_INTERVAL_MS  2000U
#endif

#endif /* STM32_AHT20_CONF_H */
