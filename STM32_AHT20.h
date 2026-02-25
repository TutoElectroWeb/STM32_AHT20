/**
 *******************************************************************************
 * @file    STM32_AHT20.h
 * @author  STM32_LIB_STYLE_GUIDE
 * @brief   Driver pour le capteur AHT20 (Aosong) — température et humidité I2C
 * @version 0.9.0
 * @date    2026-02-22
 * @copyright Libre sous licence MIT.
 * @note    Délais bloquants : SoftReset 20ms, Init cmd 20ms, ReadMeasurements 80ms
 * @note    Compatibilité RTOS : HAL_Delay() suspend la tâche (pas de spin-wait) ✅
 *          async_busy/async_state sont volatile : safe lecture multi-tâche.
 *          Pour plusieurs tâches accédant au bus : mutex I2C recommandé.
 * @note    sizeof(AHT20_Handle_t) ≈ 32 octets
 *******************************************************************************
 */
#ifndef STM32_AHT20_H
#define STM32_AHT20_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ============================================================================ */

#include "main.h"              ///< Portabilité multi-famille STM32 (inclut le bon HAL via CubeMX)
#include <stdbool.h>           ///< Type bool, true, false
#include "STM32_AHT20_conf.h"  ///< Configuration overridable (timeouts, MAX_CONSECUTIVE_ERRORS, DEBUG)

/* ============================================================================
 * Exported constants
 * ============================================================================ */

/* Version du driver */
#define AHT20_VERSION_MAJOR 0   ///< Version majeure de la lib
#define AHT20_VERSION_MINOR 9   ///< Version mineure de la lib
#define AHT20_VERSION_PATCH 0   ///< Version patch de la lib

/* Adresse I2C du capteur AHT20 (profil SINGLE_ADDRESS) */
#define AHT20_I2C_ADDR_7B 0x38         ///< Adresse unique 7-bit

/* Commandes AHT20 */
#define AHT20_CMD_INIT        0xBE  ///< Commande d'initialisation
#define AHT20_CMD_TRIGGER     0xAC  ///< Commande de déclenchement de la mesure
#define AHT20_CMD_SOFT_RESET  0xBA  ///< Commande de reset logiciel

/* Paramètres des commandes (datasheet §5.4) */
#define AHT20_INIT_PARAM1     0x08  ///< Premier paramètre commande init (0xBE)
#define AHT20_INIT_PARAM2     0x00  ///< Second paramètre commande init
#define AHT20_MEASURE_PARAM1  0x33  ///< Premier paramètre commande trigger (0xAC)
#define AHT20_MEASURE_PARAM2  0x00  ///< Second paramètre commande trigger

/* Délais typiques (en ms) et timeout I2C — définis dans STM32_AHT20_conf.h avec #ifndef
 * (surchargeables via -DAHT20_I2C_TIMEOUT_MS=X ou définition AVANT l'include) */
// AHT20_I2C_TIMEOUT_MS          → conf.h (défaut 100 ms)
// AHT20_DELAY_SOFT_RESET_MS     → conf.h (défaut 20 ms, datasheet ≤20ms)
// AHT20_DELAY_INIT_CMD_WAIT_MS  → conf.h (défaut 20 ms, datasheet 10ms min)
// AHT20_DELAY_MEASUREMENT_WAIT_MS→ conf.h (défaut 80 ms, datasheet ≥75ms)

/* ============================================================================
 * Exported types
 * ============================================================================ */

/* Structure pour contenir les données du capteur */
typedef struct
{
    float temperature;  ///< Température en °C
    float humidity;     ///< Humidité relative en %
} AHT20_Data;

/* Codes d'erreur — définis AVANT AHT20_Handle (used dans le champ last_error) */
typedef enum
{
    AHT20_OK = 0,                 ///< Opération réussie
    AHT20_ERR_I2C,               ///< Erreur I2C (HAL_ERROR/TIMEOUT/BUSY)
    AHT20_ERR_CHECKSUM,          ///< CRC-8 invalide
    AHT20_ERR_TIMEOUT,           ///< Timeout mesure
    AHT20_ERR_CALIBRATION,       ///< Bit CAL Enable non actif (capteur non calibré)
    AHT20_ERR_BUSY,              ///< Capteur occupé (bit Busy actif)
    AHT20_ERR_NULL_PTR,          ///< Pointeur NULL passé en argument
    AHT20_ERR_INVALID_PARAM,     ///< Paramètre invalide (valeur hors plage)
    AHT20_ERR_NOT_INITIALIZED,   ///< Handle non initialisé (AHT20_Init non appelé)
    AHT20_ERR_NOT_CONFIGURED,    ///< Capteur non configuré pour la mesure
    AHT20_ERR_OVERFLOW,          ///< Débordement buffer interne (multi-capteurs)
} AHT20_Status;

/**
 * @brief Handle AHT20
 * @note  sizeof(AHT20_Handle_t) ≈ 32 octets
 * @note  Champs async_busy et consecutive_errors modifiés depuis IRQ → déclarés volatile
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;           ///< Pointeur I2C HAL
    uint16_t           i2c_addr;       ///< Adresse I2C shiftée ×2 pour HAL
    uint32_t           i2c_timeout;    ///< Timeout HAL en ms
    bool               initialized;    ///< true après Init réussi

    uint32_t           sample_interval_ms; ///< Intervalle minimal entre mesures async (ms)

    volatile uint8_t   consecutive_errors; ///< Compteur erreurs consécutives (reset à 0 si OK)
    volatile AHT20_Status last_error;      ///< Dernier code d'erreur (modifiable depuis IRQ → volatile)
    uint32_t           last_hal_error;     ///< Dernier HAL_I2C_GetError() brut

    volatile uint8_t   async_busy;         ///< 1 = transfert IT en cours (guard polling)
} AHT20_Handle_t;


/* ============================================================================
 * Exported functions prototypes
 * ============================================================================ */

/**
 * @brief  Initialise le capteur AHT20 (soft reset + lecture statut + cmd init si CAL=0)
 * @note   Séquence conforme datasheet §5.4.1 : lit le statut AVANT d'envoyer 0xBE,
 *         n'envoie la commande d'init que si le bit CAL (bit [3]) est à 0.
 * @param  haht20  Handle AHT20 (non NULL, non initialisé)
 * @param  hi2c    Handle I2C HAL (non NULL)
 * @retval AHT20_OK              Succès — handle marqué initialized
 * @retval AHT20_ERR_NULL_PTR   haht20 ou hi2c est NULL
 * @retval AHT20_ERR_I2C        Erreur bus I2C
 * @retval AHT20_ERR_CALIBRATION Bit CAL non actif après commande d'init
 * @retval AHT20_ERR_BUSY       Capteur occupé après init
 */
AHT20_Status AHT20_Init(AHT20_Handle_t *haht20, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Remet le handle à zéro (initialized = false, tous les champs = 0).
 * @note   Indispensable avant un re-Init, pour un bootloader, ou les tests unitaires.
 *         N'envoie aucune commande sur le bus. Safe si haht20 est NULL (retourne ERR_NULL_PTR).
 * @param  haht20  Handle AHT20 (peut être en cours d'utilisation ou déjà erroné)
 * @retval AHT20_OK           Succès
 * @retval AHT20_ERR_NULL_PTR haht20 est NULL
 */
AHT20_Status AHT20_DeInit(AHT20_Handle_t *haht20);

/**
 * @brief  Effectue un reset logiciel (commande 0xBA) et attend AHT20_DELAY_SOFT_RESET_MS.
 * @param  haht20  Handle AHT20 initialisé (non NULL)
 * @retval AHT20_OK            Succès
 * @retval AHT20_ERR_NULL_PTR  haht20 ou hi2c est NULL
 * @retval AHT20_ERR_I2C       Erreur bus I2C
 */
AHT20_Status AHT20_SoftReset(AHT20_Handle_t *haht20);

/**
 * @brief  Déclenche une mesure et lit les données (bloquant ~80ms).
 * @note   Envoie 0xAC 0x33 0x00, attend AHT20_DELAY_MEASUREMENT_WAIT_MS, lit 7 octets,
 *         vérifie le CRC-8 (poly 0x31, init 0xFF), décode température et humidité.
 * @param  haht20  Handle AHT20 initialisé (non NULL)
 * @param  data    Pointeur vers AHT20_Data pour stocker la mesure (non NULL)
 * @retval AHT20_OK               Succès — data.temperature et data.humidity remplis
 * @retval AHT20_ERR_NULL_PTR    haht20 ou data est NULL
 * @retval AHT20_ERR_NOT_INITIALIZED Handle non initialisé
 * @retval AHT20_ERR_I2C          Erreur bus I2C
 * @retval AHT20_ERR_BUSY         Capteur occupé après délai mesure
 * @retval AHT20_ERR_CHECKSUM     CRC invalide
 */
AHT20_Status AHT20_ReadMeasurements(AHT20_Handle_t *haht20, AHT20_Data *data);

/**
 * @brief  Lit l'octet de statut du capteur (HAL_I2C_Master_Receive — 1 octet).
 * @param  haht20  Handle AHT20 initialisé (non NULL)
 * @param  status  Pointeur pour recevoir l'octet de statut (non NULL)
 *                 Bit [7] = Busy, Bit [3] = CAL Enable
 * @retval AHT20_OK               Succès
 * @retval AHT20_ERR_NULL_PTR    Pointeur NULL
 * @retval AHT20_ERR_NOT_INITIALIZED Handle non initialisé
 * @retval AHT20_ERR_I2C          Erreur bus I2C
 */
AHT20_Status AHT20_GetStatus(AHT20_Handle_t *haht20, uint8_t *status);

/**
 * @brief  Convertit un code AHT20_Status en chaîne de caractères (pour debug).
 * @note   Retourne "" si AHT20_DEBUG_ENABLE n'est pas défini (body conditionnel dans le .c).
 *         Table de chaînes exclue de la flash en production.
 * @param  status  Code retour à convertir
 * @retval Pointeur vers une chaîne statique constante (jamais NULL)
 */
const char *AHT20_StatusToString(AHT20_Status status);

/**
 * @brief  Retourne le dernier code d'erreur mémorisé dans le handle.
 * @param  haht20  Handle AHT20 (non NULL)
 * @retval Dernier code d'erreur, ou AHT20_ERR_NULL_PTR si handle invalide
 */
AHT20_Status AHT20_GetLastError(const AHT20_Handle_t *haht20);

/**
 * @brief  Retourne le compteur d'erreurs consécutives.
 * @param  haht20  Handle AHT20 (non NULL)
 * @retval Nombre d'erreurs consécutives, 0 si handle invalide
 */
uint8_t AHT20_GetConsecutiveErrors(const AHT20_Handle_t *haht20);

/** @brief Alias ReadMeasurements — conforme au pattern commun des libs STM32 */
#define AHT20_ReadAll AHT20_ReadMeasurements

/* ============================================================================
 * API ASYNCHRONE (IT uniquement) - Gain CPU lors de l'attente mesure (80ms)
 * @note DMA non implémenté : trame ≤ 7 octets (aucun gain CPU) et bus I2C
 *       partagé multi-capteurs (transfert DMA atomique, arbitrage impossible).
 *       IT suffit et cohabite avec SGP40, DHT20, etc. sur le même bus.
 * ============================================================================ */

/**
 * @brief AHT20 Async state machine
 */
typedef enum {
    AHT20_ASYNC_IDLE = 0,        /**< Inactif */
    AHT20_ASYNC_TRIGGER_TX,      /**< Envoi commande trigger */
    AHT20_ASYNC_WAIT_MEAS,       /**< Attente mesure (80ms) */
    AHT20_ASYNC_DATA_RX,         /**< Lecture données */
    AHT20_ASYNC_DONE,            /**< Mesure complète */
    AHT20_ASYNC_ERROR            /**< Erreur */
} AHT20_AsyncState;

/**
 * @brief Callbacks utilisateur (contexte main loop via Process())
 */
typedef void (*AHT20_Async_OnDataReadyCb)(void *user_ctx, const AHT20_Data *data, AHT20_Status status);
typedef void (*AHT20_Async_OnErrorCb)(void *user_ctx, AHT20_Status status);

/**
 * @brief Callbacks IRQ-safe (contexte interruption, ultra-court)
 */
typedef void (*AHT20_Async_OnIrqDataReadyCb)(void *user_ctx);
typedef void (*AHT20_Async_OnIrqErrorCb)(void *user_ctx);

/**
 * @brief  Structure contexte asynchrone AHT20
 * @warning Thread model : Process/Tick/TriggerEvery depuis la tâche principale
 *          uniquement. Les callbacks on_irq_* sont appelés depuis contexte IRQ
 *          (ultra-court, sans HAL_Delay). Ne jamais appeler ces fonctions depuis
 *          plusieurs tâches simultanément sans mutex dédié.
 */
typedef struct {
    AHT20_Handle_t *haht20;         /**< Handle AHT20 */
    I2C_HandleTypeDef *hi2c;          /**< Pointeur I2C (copie depuis haht20) */
    uint16_t           i2c_addr;       ///< Adresse I2C shiftée ×2 pour HAL
    uint32_t           i2c_timeout;    ///< Timeout HAL en ms
    
    volatile AHT20_AsyncState state;
    volatile AHT20_Status last_status;
    
    uint32_t meas_deadline_ms;       /**< Deadline attente mesure */
    uint32_t i2c_deadline_ms;        /**< Deadline timeout I2C */
    
    uint8_t tx_buf[3];               /**< Buffer commande trigger */
    uint8_t rx_buf[7];               /**< Buffer données (statut+H+T+CRC) */
    AHT20_Data data;                 /**< Résultats calculés */
    
    // Flags et callbacks
    volatile bool data_ready_flag;
    volatile bool error_flag;
    volatile bool notify_data_pending;   ///< Écrit en IRQ, lu en main-loop → volatile
    volatile bool notify_error_pending;  ///< Écrit en IRQ, lu en main-loop → volatile

    AHT20_Async_OnDataReadyCb on_data_ready;
    AHT20_Async_OnErrorCb on_error;
    AHT20_Async_OnIrqDataReadyCb on_irq_data_ready;
    AHT20_Async_OnIrqErrorCb on_irq_error;
    void *user_ctx;      ///< Contexte main-loop (SetCallbacks)
    void *irq_user_ctx;  ///< Contexte IRQ-safe distinct (SetIrqCallbacks) — ne jamais écraser user_ctx
} AHT20_Async;

/* Fonctions API asynchrone */

/**
 * @brief  Initialise le contexte asynchrone
 * @param  ctx     Pointeur vers le contexte async (non NULL)
 * @param  haht20  Pointeur vers le handle AHT20 (non NULL)
 * @retval None
 */
void AHT20_Async_Init(AHT20_Async *ctx, AHT20_Handle_t *haht20);

/**
 * @brief Réinitialise la machine d'état async sans perdre les callbacks
 * @param ctx Contexte async (non NULL)
 * @note Préserve : haht20, hi2c, tous les callbacks, user_ctx.
 *       Remet : state=IDLE, flags=false, buffers=0, deadlines=0.
 *       Utile pour récupération d'erreur sans ré-appeler Init+SetCallbacks.
  * @pre  ctx non NULL
 * @post ctx->state == AHT20_ASYNC_IDLE
 * @post ctx->haht20->async_busy == 0  (libère le guard polling)
 * @warning ⚠️ Abandonne toute mesure en cours — appeler depuis main loop uniquement
 */
void AHT20_Async_Reset(AHT20_Async *ctx);

/**
 * @brief Configure les callbacks utilisateur (appel depuis Process)
 */
void AHT20_Async_SetCallbacks(AHT20_Async *ctx,
                              AHT20_Async_OnDataReadyCb on_data_ready,
                              AHT20_Async_OnErrorCb on_error,
                              void *user_ctx);

/**
 * @brief Configure les callbacks IRQ-safe
 */
void AHT20_Async_SetIrqCallbacks(AHT20_Async *ctx,
                                 AHT20_Async_OnIrqDataReadyCb on_irq_data_ready,
                                 AHT20_Async_OnIrqErrorCb on_irq_error,
                                 void *user_ctx);

/**
 * @brief Déclenche une lecture asynchrone en mode IT
 * @note  IT uniquement — DMA non implémenté (trame 7B, bus I2C partagé).
 * @return AHT20_OK si démarré, erreur sinon
  * @pre  ctx->haht20->initialized == true  (AHT20_Init() réussi)
 * @pre  NVIC activé pour le périphérique I2C utilisé
 * @pre  HAL_GetTick() monotone (uint32_t, wraparound géré par soustraction non signée)
 * @post ctx->state transitoire après déclenchement (ou ASYNC_IDLE si HAL_BUSY — normal sur bus partagé)
 */
AHT20_Status AHT20_ReadAll_IT(AHT20_Async *ctx);

/**
 * @brief Machine à états asynchrone (appeler dans main loop)
 * @param now_ms Timestamp courant (HAL_GetTick())
 */
void AHT20_Async_Process(AHT20_Async *ctx, uint32_t now_ms);

/**
 * @brief Vérifie si contexte est inactif
 */
bool AHT20_Async_IsIdle(const AHT20_Async *ctx);

/**
 * @brief Vérifie si données disponibles
 */
bool AHT20_Async_HasData(const AHT20_Async *ctx);

/**
 * @brief Récupère les dernières données
 * @note  Cette fonction consomme la mesure et remet l'état à AHT20_ASYNC_IDLE.
 *        Elle doit être appelée après DataReadyFlag/IRQ pour autoriser la mesure suivante.
 */
AHT20_Status AHT20_Async_GetData(AHT20_Async *ctx, AHT20_Data *out);

/**
 * @brief Vérification flag data_ready
 */
bool AHT20_Async_DataReadyFlag(const AHT20_Async *ctx);

/**
 * @brief Vérification flag error
 */
bool AHT20_Async_ErrorFlag(const AHT20_Async *ctx);

/**
 * @brief Clear les flags
 */
void AHT20_Async_ClearFlags(AHT20_Async *ctx);

/* Callbacks HAL à appeler depuis stm32l4xx_it.c */

/**
 * @brief À appeler depuis HAL_I2C_MasterTxCpltCallback()
  * @pre  Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_MasterTxCpltCallback)
 */
void AHT20_Async_OnI2CMasterTxCplt(AHT20_Async *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief À appeler depuis HAL_I2C_MasterRxCpltCallback()
  * @pre  Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_MasterRxCpltCallback)
 * @pre  rx_buf dimensionné à ≥ 7 octets (taille trame protocole)
 * @post notify_data_pending == true (si CRC/parité OK) ou notify_error_pending == true
 */
void AHT20_Async_OnI2CMasterRxCplt(AHT20_Async *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief À appeler depuis HAL_I2C_ErrorCallback()
  * @pre  Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_ErrorCallback)
 * @post notify_error_pending == true si handle correspond
 */
void AHT20_Async_OnI2CError(AHT20_Async *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief Résultat court d'un appel AHT20_Async_Tick()
 */
typedef enum {
    AHT20_TICK_IDLE       = 0, /**< Contexte inactif */
    AHT20_TICK_BUSY       = 1, /**< Mesure en cours */
    AHT20_TICK_DATA_READY = 2, /**< Donnée disponible dans *data_out */
    AHT20_TICK_ERROR      = 3  /**< Erreur — contexte réinitialisé */
} AHT20_TickResult;

/**
 * @brief Avance la FSM async et retourne un résultat actionnable
 * @param ctx      Contexte async (non NULL)
 * @param now_ms   Timestamp HAL_GetTick()
 * @param data_out Pointeur pour recevoir la mesure (peut être NULL)
 * @retval AHT20_TickResult
 */
AHT20_TickResult AHT20_Async_Tick(AHT20_Async *ctx, uint32_t now_ms, AHT20_Data *data_out);

/**
 * @brief Déclenche une mesure IT à intervalle régulier
 * @param ctx     Contexte async (non NULL)
 * @param now_ms  Timestamp HAL_GetTick()
 * @param last_ms Pointeur vers le timestamp de la dernière mesure
 * @retval AHT20_OK si déclenchée ou intervalle non écoulé, AHT20_ERR_BUSY si FSM occupée
 * @note  L'intervalle est lu depuis ctx->haht20->sample_interval_ms
 *        (initialisé à AHT20_DEFAULT_SAMPLE_INTERVAL_MS dans AHT20_Init).
 */
AHT20_Status AHT20_Async_TriggerEvery(AHT20_Async *ctx, uint32_t now_ms,
                                       uint32_t *last_ms);

#ifdef __cplusplus
}
#endif

#endif /* STM32_AHT20_H */
