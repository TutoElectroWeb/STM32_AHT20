/**
  ******************************************************************************
  * @file    STM32_AHT20.c
  * @brief   Implémentation driver STM32 HAL pour capteur T/H AHT20 (Aosong)
  * @author  STM32_LIB_STYLE_GUIDE
  * @date    2026-02-22
  * @version 0.9.0
  ******************************************************************************
  */

#include "STM32_AHT20.h"

/* ============================================================================
 * Private function prototypes
 * ============================================================================ */
static uint8_t AHT20_CalculateChecksum(uint8_t *data, uint8_t len);

/* ============================================================================
 * Exported functions - Helpers
 * ============================================================================ */

#ifdef AHT20_DEBUG_ENABLE
/**
 * @brief Convertit un code de statut en chaîne pour debug
 * @note  Disponible uniquement si AHT20_DEBUG_ENABLE est défini (voir STM32_AHT20_conf.h)
 * @param status Code de statut AHT20
 * @return Chaîne descriptive (jamais NULL)
 */
const char* AHT20_StatusToString(AHT20_Status status) {
    switch (status) {
        case AHT20_OK:                    return "OK";
        case AHT20_ERR_I2C:               return "ERR_I2C";
        case AHT20_ERR_CHECKSUM:          return "ERR_CHECKSUM";
        case AHT20_ERR_TIMEOUT:           return "ERR_TIMEOUT";
        case AHT20_ERR_CALIBRATION:       return "ERR_CALIBRATION";
        case AHT20_ERR_BUSY:              return "ERR_BUSY";
        case AHT20_ERR_NULL_PTR:          return "ERR_NULL_PTR";
        case AHT20_ERR_INVALID_PARAM:     return "ERR_INVALID_PARAM";
        case AHT20_ERR_NOT_INITIALIZED:   return "ERR_NOT_INITIALIZED";
        case AHT20_ERR_NOT_CONFIGURED:    return "ERR_NOT_CONFIGURED";
        case AHT20_ERR_OVERFLOW:          return "ERR_OVERFLOW";
        default:                          return "UNKNOWN";
    }
}
#endif /* AHT20_DEBUG_ENABLE */

/* ============================================================================
 * Private functions
 * ============================================================================ */

/**
 * @brief Calcule le CRC-8 sur un tableau d'octets (algorithme AHT20)
 * @param data Pointeur vers les données à vérifier
 * @param len  Nombre d'octets à traiter
 * @retval CRC-8 calculé (polynome 0x31, init 0xFF)
 * @note CRC initial 0xFF, polynome x^8 + x^5 + x^4 + 1 (0x31), identique Sensirion.
 */
static uint8_t AHT20_CalculateChecksum(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;                    // Valeur initiale CRC (datasheet §5.4)
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31; // Bit fort à 1 : décale + applique polynom 0x31
            } else {
                crc <<= 1;               // Bit fort à 0 : décale uniquement
            }
        }
    }
    return crc;
}

/* ============================================================================
 * Exported functions - Main API
 * ============================================================================ */

/**
 * @brief Initialise le capteur AHT20
 * @param haht20 Pointeur vers le handle AHT20
 * @param hi2c Pointeur vers la structure de handle I2C
 * @return Statut de l'opération
 */
AHT20_Status AHT20_Init(AHT20_Handle *haht20, I2C_HandleTypeDef *hi2c) {
    uint8_t cmd_init[3] = {AHT20_CMD_INIT, AHT20_INIT_PARAM1, AHT20_INIT_PARAM2};
    uint8_t status_byte;
    HAL_StatusTypeDef hal_status;

    if (haht20 == NULL || hi2c == NULL) {
        return AHT20_ERR_NULL_PTR;
    }

    /* Initialisation du handle */
    haht20->hi2c               = hi2c;
    haht20->i2c_addr           = (uint16_t)(AHT20_I2C_ADDR_7B << 1);
    haht20->i2c_timeout        = AHT20_I2C_TIMEOUT_MS;
    haht20->initialized        = false;
    haht20->sample_interval_ms = AHT20_DEFAULT_SAMPLE_INTERVAL_MS;
    haht20->consecutive_errors = 0;
    haht20->last_error         = AHT20_OK;
    haht20->last_hal_error     = 0;
    haht20->async_busy         = 0;

    /* Reset logiciel — contient HAL_Delay(AHT20_DELAY_SOFT_RESET_MS = 20ms) */
    AHT20_Status reset_st = AHT20_SoftReset(haht20);
    if (reset_st != AHT20_OK) {
        return reset_st;
    }

    /* Lire statut directement (initialized=false ici, garde AHT20_GetStatus non applicable).
     * Datasheet §5.4.1 : lire statut après power-on/reset, envoyer 0xBE uniquement si CAL=0. */
    hal_status = HAL_I2C_Master_Receive(haht20->hi2c, haht20->i2c_addr,
                                        &status_byte, 1, haht20->i2c_timeout);
    if (hal_status != HAL_OK) {
        return AHT20_ERR_I2C;
    }

    /* Envoyer la commande d'init uniquement si CAL=0 (bit [3]) — datasheet §5.4.1 */
    if (!(status_byte & 0x08)) {
        hal_status = HAL_I2C_Master_Transmit(haht20->hi2c, haht20->i2c_addr,
                                             cmd_init, 3, haht20->i2c_timeout);
        if (hal_status != HAL_OK) {
            return AHT20_ERR_I2C;
        }
        /* Attendre fin init (datasheet: 10ms min — on utilise 20ms pour sécurité) */
        HAL_Delay(AHT20_DELAY_INIT_CMD_WAIT_MS);

        /* Re-lire statut pour vérifier que la calibration est maintenant active */
        hal_status = HAL_I2C_Master_Receive(haht20->hi2c, haht20->i2c_addr,
                                            &status_byte, 1, haht20->i2c_timeout);
        if (hal_status != HAL_OK) {
            return AHT20_ERR_I2C;
        }
        if (!(status_byte & 0x08)) { /* CAL toujours 0 après init = défaillance capteur */
            return AHT20_ERR_CALIBRATION;
        }
    }

    /* Vérifier que le capteur n'est pas occupé (bit [7]) */
    if (status_byte & 0x80) {
        return AHT20_ERR_BUSY;
    }

    haht20->initialized = true;
    haht20->consecutive_errors = 0;
    haht20->last_error         = AHT20_OK;
    return AHT20_OK;
}

/**
 * @brief Remet le handle à zéro (DeInit)
 */
AHT20_Status AHT20_DeInit(AHT20_Handle *haht20) {
    if (haht20 == NULL) {
        return AHT20_ERR_NULL_PTR;
    }
    if (haht20->async_busy) {
        return AHT20_ERR_BUSY;  /* Transfert IT en cours — DeInit refusé */
    }
    /* Mise à zéro complète — hi2c = NULL, initialized = false */
    haht20->hi2c               = NULL;
    haht20->i2c_addr           = 0;
    haht20->i2c_timeout        = 0;
    haht20->initialized        = false;
    haht20->consecutive_errors = 0;
    haht20->last_error         = AHT20_OK;
    haht20->last_hal_error     = 0;
    haht20->async_busy         = 0;
    return AHT20_OK;
}

/**
 * @brief Effectue un reset logiciel du capteur
 * @param haht20 Pointeur vers le handle AHT20
 * @return Statut de l'opération
 */
AHT20_Status AHT20_SoftReset(AHT20_Handle *haht20) {
    uint8_t cmd = AHT20_CMD_SOFT_RESET;
    HAL_StatusTypeDef hal_status;

    if (haht20 == NULL || haht20->hi2c == NULL) {
        return AHT20_ERR_NULL_PTR;
    }
    /* Guard : rejeter si une transaction async est en cours */
    if (haht20->async_busy) {
        return AHT20_ERR_BUSY;
    }

    hal_status = HAL_I2C_Master_Transmit(haht20->hi2c, haht20->i2c_addr, &cmd, 1, haht20->i2c_timeout);
    if (hal_status != HAL_OK) {
        haht20->consecutive_errors++;
        haht20->last_error     = AHT20_ERR_I2C;
        haht20->last_hal_error = HAL_I2C_GetError(haht20->hi2c);
        return AHT20_ERR_I2C;
    }

    /* Attendre que le reset soit terminé (datasheet : ≤20ms) */
    HAL_Delay(AHT20_DELAY_SOFT_RESET_MS);
    haht20->consecutive_errors = 0;
    return AHT20_OK;
}

/**
 * @brief Récupère le statut du capteur
 * @param haht20 Pointeur vers le handle AHT20
 * @param status Pointeur pour stocker l'octet de statut
 * @return Statut de l'opération
 */
AHT20_Status AHT20_GetStatus(AHT20_Handle *haht20, uint8_t *status) {
    HAL_StatusTypeDef hal_status;

    if (haht20 == NULL || haht20->hi2c == NULL || status == NULL) {
        return AHT20_ERR_NULL_PTR;
    }
    if (!haht20->initialized) {
        return AHT20_ERR_NOT_INITIALIZED;
    }
    /* Guard : rejeter si une transaction async est en cours */
    if (haht20->async_busy) {
        return AHT20_ERR_BUSY;
    }

    hal_status = HAL_I2C_Master_Receive(haht20->hi2c, haht20->i2c_addr, status, 1, haht20->i2c_timeout);
    if (hal_status != HAL_OK) {
        haht20->consecutive_errors++;
        haht20->last_error     = AHT20_ERR_I2C;
        haht20->last_hal_error = HAL_I2C_GetError(haht20->hi2c);
        return AHT20_ERR_I2C;
    }

    haht20->consecutive_errors = 0;
    return AHT20_OK;
}

/**
 * @brief Lit les mesures de température et d'humidité
 * @param haht20 Pointeur vers le handle AHT20
 * @param data Pointeur vers une structure pour stocker les mesures
 * @return Statut de l'opération
 */
AHT20_Status AHT20_ReadMeasurements(AHT20_Handle *haht20, AHT20_Data *data) {
    uint8_t cmd[3] = {AHT20_CMD_TRIGGER, AHT20_MEASURE_PARAM1, AHT20_MEASURE_PARAM2};
    uint8_t buffer[7];
    HAL_StatusTypeDef hal_status;
    uint32_t raw_humidity, raw_temperature;

    if (haht20 == NULL || data == NULL) {
        return AHT20_ERR_NULL_PTR;
    }
    if (!haht20->initialized) {
        return AHT20_ERR_NOT_INITIALIZED;
    }
    /* Guard : rejeter si une transaction async est en cours */
    if (haht20->async_busy) {
        return AHT20_ERR_BUSY;
    }

    /* Envoyer la commande de mesure */
    hal_status = HAL_I2C_Master_Transmit(haht20->hi2c, haht20->i2c_addr, cmd, 3, haht20->i2c_timeout);
    if (hal_status != HAL_OK) {
        haht20->consecutive_errors++;
        haht20->last_error     = AHT20_ERR_I2C;
        haht20->last_hal_error = HAL_I2C_GetError(haht20->hi2c);
        return AHT20_ERR_I2C;
    }

    /* Attendre la fin de la mesure (datasheet: ≥75ms, on utilise 80ms) */
    HAL_Delay(AHT20_DELAY_MEASUREMENT_WAIT_MS);

    /* Lire les données (7 octets : Statut, H1, H2, H3, T1, T2, CRC) */
    hal_status = HAL_I2C_Master_Receive(haht20->hi2c, haht20->i2c_addr, buffer, 7, haht20->i2c_timeout);
    if (hal_status != HAL_OK) {
        haht20->consecutive_errors++;
        haht20->last_error     = AHT20_ERR_I2C;
        haht20->last_hal_error = HAL_I2C_GetError(haht20->hi2c);
        return AHT20_ERR_I2C;
    }

    /* Vérifier si le capteur est occupé (bit [7] de l'octet de statut) */
    if (buffer[0] & 0x80) {
        haht20->consecutive_errors++;
        haht20->last_error = AHT20_ERR_BUSY;
        return AHT20_ERR_BUSY;
    }

    /* Vérifier le checksum CRC-8 (poly 0x31, init 0xFF, sur 6 octets) */
    uint8_t calculated_crc = AHT20_CalculateChecksum(buffer, 6);
    if (calculated_crc != buffer[6]) {
        haht20->consecutive_errors++;
        haht20->last_error = AHT20_ERR_CHECKSUM;
        return AHT20_ERR_CHECKSUM;
    }

    /* Convertir les données brutes */
    raw_humidity    = (((uint32_t)buffer[1] << 12) | ((uint32_t)buffer[2] << 4) | (buffer[3] >> 4));
    raw_temperature = (((uint32_t)buffer[3] & 0x0F) << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];

    data->humidity    = (raw_humidity    * 100.0f) / 1048576.0f;          /* RH = (S_RH / 2^20) * 100 */
    data->temperature = ((raw_temperature * 200.0f) / 1048576.0f) - 50.0f; /* T  = (S_T  / 2^20) * 200 - 50 */

    haht20->consecutive_errors = 0;
    haht20->last_error         = AHT20_OK;
    return AHT20_OK;
}

/* ============================================================================
 * Exported functions — API Asynchrone (IT uniquement)
 * @note DMA non implémenté : trame ≤ 7 octets (aucun gain CPU) et bus I2C
 *       partagé multi-capteurs (transfert DMA atomique, arbitrage impossible).
 * ============================================================================ */

/**
 * @brief Initialise le contexte asynchrone
 */
void AHT20_Async_Init(AHT20_Async *ctx, AHT20_Handle *haht20) {
    if (ctx == NULL || haht20 == NULL) return;
    if (!haht20->initialized) {
        ctx->last_status = AHT20_ERR_NOT_INITIALIZED;
        ctx->state = AHT20_ASYNC_ERROR;
        return;
    }
    
    ctx->haht20 = haht20;
    ctx->hi2c = haht20->hi2c;  /* Copie pour compatibilité */
    ctx->i2c_addr = haht20->i2c_addr;
    ctx->i2c_timeout = haht20->i2c_timeout;
    ctx->state = AHT20_ASYNC_IDLE;
    ctx->last_status = AHT20_OK;
    ctx->meas_deadline_ms = 0;
    ctx->i2c_deadline_ms = 0;
    
    ctx->data_ready_flag = false;
    ctx->error_flag = false;
    ctx->notify_data_pending = false;
    ctx->notify_error_pending = false;
    
    ctx->on_data_ready = NULL;
    ctx->on_error = NULL;
    ctx->on_irq_data_ready = NULL;
    ctx->on_irq_error = NULL;
    ctx->user_ctx = NULL;
}

/**
 * @brief Réinitialise la machine d'état async sans perdre les callbacks
 */
void AHT20_Async_Reset(AHT20_Async *ctx) {
    if (ctx == NULL) return;

    ctx->state               = AHT20_ASYNC_IDLE;    // Remet à l'état repos
    ctx->last_status         = AHT20_OK;             // Efface le dernier code erreur
    ctx->meas_deadline_ms    = 0;                    // Réinitialise deadline mesure
    ctx->i2c_deadline_ms     = 0;                    // Réinitialise deadline I2C
    ctx->data_ready_flag     = false;                // Efface flag données prêtes
    ctx->error_flag          = false;                // Efface flag erreur
    ctx->notify_data_pending = false;                // Annule notification data en attente
    ctx->notify_error_pending = false;               // Annule notification erreur en attente

    /* Callbacks, user_ctx, haht20/hi2c et mode sont préservés */
}

/**
 * @brief Configure les callbacks utilisateur (contexte main loop)
 */
void AHT20_Async_SetCallbacks(AHT20_Async *ctx,
                              AHT20_Async_OnDataReadyCb on_data_ready,
                              AHT20_Async_OnErrorCb on_error,
                              void *user_ctx) {
    if (ctx == NULL) return;
    ctx->on_data_ready = on_data_ready;
    ctx->on_error = on_error;
    ctx->user_ctx = user_ctx;
}

/**
 * @brief Configure les callbacks IRQ-safe
 */
void AHT20_Async_SetIrqCallbacks(AHT20_Async *ctx,
                                 AHT20_Async_OnIrqDataReadyCb on_irq_data_ready,
                                 AHT20_Async_OnIrqErrorCb on_irq_error,
                                 void *user_ctx) {
    if (ctx == NULL) return;
    ctx->on_irq_data_ready = on_irq_data_ready;
    ctx->on_irq_error = on_irq_error;
    ctx->user_ctx = user_ctx;
}

/**
 * @brief Déclenche lecture asynchrone mode IT
 */
AHT20_Status AHT20_ReadAll_IT(AHT20_Async *ctx) {
    if (ctx == NULL || ctx->hi2c == NULL) {
        return AHT20_ERR_NULL_PTR;
    }
    if (!ctx->haht20->initialized) {
        return AHT20_ERR_NOT_INITIALIZED;
    }
    
    if (ctx->state != AHT20_ASYNC_IDLE) {
        return AHT20_ERR_BUSY;
    }

    /* Prépare commande trigger */
    ctx->tx_buf[0] = AHT20_CMD_TRIGGER;
    ctx->tx_buf[1] = AHT20_MEASURE_PARAM1;
    ctx->tx_buf[2] = AHT20_MEASURE_PARAM2;

    ctx->state = AHT20_ASYNC_TRIGGER_TX;
    ctx->last_status = AHT20_OK;
    ctx->i2c_deadline_ms = HAL_GetTick() + ctx->i2c_timeout;

    /* Marque le bus occupé AVANT le lancement IT (guard polling) */
    ctx->haht20->async_busy = 1;

    /* Lance transmission IT */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit_IT(ctx->hi2c, ctx->i2c_addr, ctx->tx_buf, 3);

    if (hal_status != HAL_OK) {
        /* HAL_BUSY = bus occupé par un autre esclave : rester IDLE pour retry
         * lors du prochain TriggerEvery. HAL_ERROR = erreur matérielle. */
        if (hal_status == HAL_BUSY) {
            ctx->state             = AHT20_ASYNC_IDLE;
            ctx->last_status       = AHT20_ERR_BUSY;
            ctx->haht20->async_busy = 0;  /* Pas parti → libérer le guard */
            return AHT20_ERR_BUSY;
        }
        ctx->state             = AHT20_ASYNC_ERROR;
        ctx->haht20->async_busy = 0;
        ctx->haht20->consecutive_errors++;
        ctx->haht20->last_error     = AHT20_ERR_I2C;
        ctx->haht20->last_hal_error = HAL_I2C_GetError(ctx->hi2c);
        if (hal_status == HAL_TIMEOUT) {
            ctx->last_status = AHT20_ERR_TIMEOUT;
            return AHT20_ERR_TIMEOUT;
        }
        ctx->last_status = AHT20_ERR_I2C;
        return AHT20_ERR_I2C;
    }

    return AHT20_OK;
}

/**
 * @brief Machine à états asynchrone (main loop)
 */
void AHT20_Async_Process(AHT20_Async *ctx, uint32_t now_ms) {
    if (ctx == NULL) return;
    
    // Gestion notifications callbacks utilisateur
    if (ctx->notify_data_pending) {
        ctx->notify_data_pending = false;
        if (ctx->on_data_ready) {
            ctx->on_data_ready(ctx->user_ctx, &ctx->data, ctx->last_status);
        }
        return;
    }
    
    if (ctx->notify_error_pending) {
        ctx->notify_error_pending = false;
        if (ctx->on_error) {
            ctx->on_error(ctx->user_ctx, ctx->last_status);
        }
        return;
    }
    
    // Machine à états
    switch (ctx->state) {
        case AHT20_ASYNC_IDLE:
        case AHT20_ASYNC_DONE:
        case AHT20_ASYNC_ERROR:
            // Rien à faire
            break;
            
        case AHT20_ASYNC_TRIGGER_TX:
            // Attente callback HAL (TxCplt ou Error)
            // Timeout I2C (comparaison signée pour gestion wrap-around uint32)
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {
                ctx->state = AHT20_ASYNC_ERROR;
                ctx->last_status = AHT20_ERR_TIMEOUT;
                ctx->error_flag = true;
            }
            break;
            
        case AHT20_ASYNC_WAIT_MEAS:
            // Attente délai mesure (80ms)
            if ((int32_t)(now_ms - ctx->meas_deadline_ms) >= 0) {
                // Lance réception données
                ctx->state = AHT20_ASYNC_DATA_RX;
                ctx->i2c_deadline_ms = now_ms + ctx->i2c_timeout;
                
                HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive_IT(ctx->hi2c, ctx->i2c_addr, ctx->rx_buf, 7);
                
                if (hal_status != HAL_OK) {
                    ctx->state = AHT20_ASYNC_ERROR;
                    if (hal_status == HAL_BUSY) {
                        ctx->last_status = AHT20_ERR_BUSY;
                    } else if (hal_status == HAL_TIMEOUT) {
                        ctx->last_status = AHT20_ERR_TIMEOUT;
                    } else {
                        ctx->last_status = AHT20_ERR_I2C;
                    }
                    ctx->error_flag = true;
                }
            }
            break;
            
        case AHT20_ASYNC_DATA_RX:
            // Attente callback HAL (RxCplt ou Error)
            // Timeout I2C (comparaison signée pour gestion wrap-around uint32)
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {
                ctx->state = AHT20_ASYNC_ERROR;
                ctx->last_status = AHT20_ERR_TIMEOUT;
                ctx->error_flag = true;
            }
            break;
    }
}

/**
 * @brief Vérifie si inactif
 */
bool AHT20_Async_IsIdle(const AHT20_Async *ctx) {
    return (ctx != NULL) && (ctx->state == AHT20_ASYNC_IDLE);
}

/**
 * @brief Vérifie si données disponibles
 */
bool AHT20_Async_HasData(const AHT20_Async *ctx) {
    return (ctx != NULL) && (ctx->state == AHT20_ASYNC_DONE);
}

/**
 * @brief Récupère les données
 */
AHT20_Status AHT20_Async_GetData(AHT20_Async *ctx, AHT20_Data *out) {
    if (ctx == NULL || out == NULL) {
        return AHT20_ERR_NULL_PTR;
    }
    
    if (ctx->state != AHT20_ASYNC_DONE) {
        return AHT20_ERR_BUSY;
    }
    
    *out = ctx->data;
    ctx->state = AHT20_ASYNC_IDLE; // Reset state
    ctx->data_ready_flag = false;
    ctx->notify_data_pending = false;
    return ctx->last_status;
}

/**
 * @brief Vérifie flag data_ready
 */
bool AHT20_Async_DataReadyFlag(const AHT20_Async *ctx) {
    return (ctx != NULL) && ctx->data_ready_flag;
}

/**
 * @brief Vérifie flag error
 */
bool AHT20_Async_ErrorFlag(const AHT20_Async *ctx) {
    return (ctx != NULL) && ctx->error_flag;
}

/**
 * @brief Clear flags
 */
void AHT20_Async_ClearFlags(AHT20_Async *ctx) {
    if (ctx == NULL) return;
    ctx->data_ready_flag = false;
    ctx->error_flag = false;
    ctx->notify_data_pending = false;
    ctx->notify_error_pending = false;
}

/* ============================================================================
 * Exported functions — Callbacks HAL (appelés depuis stm32l4xx_it.c)
 * ============================================================================ */

/**
 * @brief Callback transmission complète (contexte IRQ)
 */
void AHT20_Async_OnI2CMasterTxCplt(AHT20_Async *ctx, I2C_HandleTypeDef *hi2c) {
    if (ctx == NULL || ctx->haht20 == NULL || ctx->hi2c != hi2c) return;
    
    if (ctx->state == AHT20_ASYNC_TRIGGER_TX) {
        // Commande trigger envoyée -> attente mesure (80ms)
        ctx->state = AHT20_ASYNC_WAIT_MEAS;
        ctx->meas_deadline_ms = HAL_GetTick() + AHT20_DELAY_MEASUREMENT_WAIT_MS;
    }
}

/**
 * @brief Callback réception complète (contexte IRQ)
 */
void AHT20_Async_OnI2CMasterRxCplt(AHT20_Async *ctx, I2C_HandleTypeDef *hi2c) {
    if (ctx == NULL || ctx->haht20 == NULL || ctx->hi2c != hi2c) return;

    if (ctx->state == AHT20_ASYNC_DATA_RX) {
        /* Vérif busy bit */
        if (ctx->rx_buf[0] & 0x80) {
            ctx->state              = AHT20_ASYNC_ERROR;
            ctx->last_status        = AHT20_ERR_BUSY;
            ctx->error_flag         = true;
            ctx->haht20->async_busy  = 0;
            ctx->haht20->consecutive_errors++;
            ctx->haht20->last_error  = AHT20_ERR_BUSY;
            if (ctx->on_irq_error) ctx->on_irq_error(ctx->user_ctx);
            ctx->notify_error_pending = true;
            return;
        }

        /* Vérif CRC-8 (poly 0x31, init 0xFF) */
        uint8_t calc_crc = AHT20_CalculateChecksum(ctx->rx_buf, 6);
        if (calc_crc != ctx->rx_buf[6]) {
            ctx->state              = AHT20_ASYNC_ERROR;
            ctx->last_status        = AHT20_ERR_CHECKSUM;
            ctx->error_flag         = true;
            ctx->haht20->async_busy  = 0;
            ctx->haht20->consecutive_errors++;
            ctx->haht20->last_error  = AHT20_ERR_CHECKSUM;
            if (ctx->on_irq_error) ctx->on_irq_error(ctx->user_ctx);
            ctx->notify_error_pending = true;
            return;
        }

        /* Conversion */
        uint32_t raw_humidity    = (((uint32_t)ctx->rx_buf[1] << 12) | ((uint32_t)ctx->rx_buf[2] << 4) | (ctx->rx_buf[3] >> 4));
        uint32_t raw_temperature = (((uint32_t)ctx->rx_buf[3] & 0x0F) << 16) | ((uint32_t)ctx->rx_buf[4] << 8) | ctx->rx_buf[5];

        ctx->data.humidity    = (raw_humidity    * 100.0f) / 1048576.0f;
        ctx->data.temperature = ((raw_temperature * 200.0f) / 1048576.0f) - 50.0f;

        ctx->state             = AHT20_ASYNC_DONE;
        ctx->last_status       = AHT20_OK;
        ctx->data_ready_flag   = true;
        ctx->haht20->async_busy = 0;                    /* Transfert terminé → libère le guard */
        ctx->haht20->consecutive_errors = 0;
        ctx->haht20->last_error = AHT20_OK;

        if (ctx->on_irq_data_ready) ctx->on_irq_data_ready(ctx->user_ctx);
        ctx->notify_data_pending = true;
    }
}

/**
 * @brief Callback erreur I2C (contexte IRQ)
 */
void AHT20_Async_OnI2CError(AHT20_Async *ctx, I2C_HandleTypeDef *hi2c) {
    if (ctx == NULL || ctx->haht20 == NULL || ctx->hi2c != hi2c) return;

    if (ctx->state != AHT20_ASYNC_IDLE && ctx->state != AHT20_ASYNC_DONE) {
        ctx->state              = AHT20_ASYNC_ERROR;
        ctx->last_status        = AHT20_ERR_I2C;
        ctx->error_flag         = true;
        ctx->haht20->async_busy  = 0;                /* Libère le guard sur erreur */
        ctx->haht20->consecutive_errors++;
        ctx->haht20->last_error     = AHT20_ERR_I2C;
        ctx->haht20->last_hal_error = HAL_I2C_GetError(hi2c);

        if (ctx->on_irq_error) ctx->on_irq_error(ctx->user_ctx);
        ctx->notify_error_pending = true;
    }
}

/* ============================================================================
 * Exported functions — Helpers Tick / TriggerEvery
 * ============================================================================ */

/**
 * @brief Avance la FSM async et retourne un résultat actionnable
 */
AHT20_TickResult AHT20_Async_Tick(AHT20_Async *ctx, uint32_t now_ms, AHT20_Data *data_out) {
    if (!ctx) return AHT20_TICK_IDLE;

    AHT20_Async_Process(ctx, now_ms);              // Avance la FSM

    if (AHT20_Async_ErrorFlag(ctx)) {              // Erreur détectée
        AHT20_Status err = ctx->last_status;
        AHT20_Async_ClearFlags(ctx);
        AHT20_Async_Reset(ctx);
        ctx->last_status = err;
        return AHT20_TICK_ERROR;
    }

    if (AHT20_Async_HasData(ctx)) {                // Donnée prête
        AHT20_Data tmp = {0};
        AHT20_Async_GetData(ctx, &tmp);
        AHT20_Async_ClearFlags(ctx);
        if (data_out) *data_out = tmp;
        return AHT20_TICK_DATA_READY;
    }

    return AHT20_Async_IsIdle(ctx) ? AHT20_TICK_IDLE : AHT20_TICK_BUSY;
}

/**
 * @brief Déclenche une mesure IT à intervalle régulier
 * @note  Retourne AHT20_OK si l'intervalle n'est pas encore écoulé (pas une
 *        erreur, appel de main loop normal) — même comportement que DHT20.
 */
AHT20_Status AHT20_Async_TriggerEvery(AHT20_Async *ctx, uint32_t now_ms,
                                       uint32_t *last_ms) {
    if (!ctx || !last_ms) return AHT20_ERR_NULL_PTR;
    if (!AHT20_Async_IsIdle(ctx)) return AHT20_ERR_BUSY;
    if ((now_ms - *last_ms) < ctx->haht20->sample_interval_ms) return AHT20_OK;  /* intervalle non écoulé */

    AHT20_Status st = AHT20_ReadAll_IT(ctx);       /* Lance la mesure IT */
    if (st == AHT20_OK) *last_ms = now_ms;
    return st;
}
