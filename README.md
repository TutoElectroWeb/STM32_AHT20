# STM32_AHT20 — Driver STM32 HAL

Driver I2C pour le capteur de température et d'humidité **AHT20** (Aosong Electronics, protocole I2C, adresse `0x38`).

**Datasheet** : [AHT20 v1.0.03](https://asairsensors.com/wp-content/uploads/2021/09/Data-Sheet-AHT20-Humidity-and-Temperature-Sensor-ASAIR-V1.0.03.pdf)

---

## Vue d'ensemble

| Paramètre         | Valeur                                                     |
| ----------------- | ---------------------------------------------------------- |
| Protocole         | I2C — adresse 7-bit `0x38` (fixe)                          |
| Plage température | −40 °C … +85 °C (±0,3 °C)                                  |
| Plage humidité    | 0 … 100 % RH (±2 % RH)                                     |
| Résolution        | 20 bits (température et humidité)                          |
| Alimentation      | 2,0 V … 5,5 V                                              |
| Durée mesure      | ~80 ms (datasheet §5.4)                                    |
| Mode transport    | `HAL_I2C_Master_Transmit_IT` / `HAL_I2C_Master_Receive_IT` |

---

## Matériel / Connexions

| Broche AHT20 | Nucleo-L476RG (I2C3) | Nucleo-L476RG (I2C1) |
| ------------ | -------------------- | -------------------- |
| VDD          | 3V3                  | 3V3                  |
| GND          | GND                  | GND                  |
| SDA          | PB11 (I2C3)          | PB7 (I2C1)           |
| SCL          | PB10 (I2C3)          | PB6 (I2C1)           |

> Pull-ups **4,7 kΩ** externes obligatoires sur SCL et SDA. Un seul jeu suffit pour tous les esclaves du bus.

---

## Pré-requis CubeMX

### Mode Polling

- I2Cx : Mode I2C, 100 kHz ou 400 kHz, Addressing 7-bit

### Mode IT (asynchrone)

- I2Cx : Mode I2C, 100 kHz ou 400 kHz, Addressing 7-bit
- NVIC → I2Cx event interrupt : **Enabled**
- NVIC → I2Cx error interrupt : **Enabled**
- USART2 : 115 200 bauds, 8N1, Asynchronous (pour `printf` via `__io_putchar`)

> **Décision IT vs DMA** : DMA non implémenté — trame AHT20 ≤ 7 octets (overhead DMA > gain CPU) et bus partagé multi-capteurs (transfert DMA atomique bloque l'arbitrage). Mode IT suffit.

---

## API (résumé)

### Initialisation / Déinitialisation

| Fonction                | Description                                                      |
| ----------------------- | ---------------------------------------------------------------- |
| `AHT20_Init(dev, hi2c)` | Init + soft reset + calibration (bloquant, ~100 ms au démarrage) |
| `AHT20_DeInit(dev)`     | Réinitialise le handle (`hi2c = NULL`, `initialized = false`)    |
| `AHT20_SoftReset(dev)`  | Reset logiciel (bloquant, ~20 ms)                                |

### API Synchrone (polling)

| Fonction                            | Description                       |
| ----------------------------------- | --------------------------------- |
| `AHT20_ReadMeasurements(dev, data)` | Lecture bloquante (~80 ms)        |
| `AHT20_GetStatus(dev, status_byte)` | Lecture registre statut (1 octet) |

### API Asynchrone IT

| Fonction                                   | Description                                                          |
| ------------------------------------------ | -------------------------------------------------------------------- |
| `AHT20_Async_Init(ctx, dev)`               | Init contexte async                                                  |
| `AHT20_Async_SetCallbacks(ctx, ...)`       | Callbacks résultat (contexte main loop)                              |
| `AHT20_Async_SetIrqCallbacks(ctx, ...)`    | Callbacks IRQ-safe (depuis ISR)                                      |
| `AHT20_ReadAll_IT(ctx)`                    | Démarrer lecture non-bloquante                                       |
| `AHT20_Async_TriggerEvery(ctx, now, last)` | Déclenchement périodique (intervalle dans `ctx->sample_interval_ms`) |
| `AHT20_Async_Process(ctx, tick)`           | FSM — appeler dans la boucle principale                              |
| `AHT20_Async_IsIdle(ctx)`                  | `true` si contexte disponible                                        |

### Callbacks HAL (USER CODE 4 — dispatcher)

```c
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    AHT20_Async_OnI2CMasterTxCplt(&aht20_ctx, hi2c);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    AHT20_Async_OnI2CMasterRxCplt(&aht20_ctx, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    AHT20_Async_OnI2CError(&aht20_ctx, hi2c);
}
```

### Debug (conditionnel)

```c
#define AHT20_DEBUG_ENABLE       // dans STM32_AHT20_conf.h ou avant l'include
const char *AHT20_StatusToString(AHT20_Status s);
```

---

## Modèle d'erreur

| Code                      | Cause                                       |
| ------------------------- | ------------------------------------------- |
| `AHT20_OK`                | Succès                                      |
| `AHT20_ERR_I2C`           | Erreur bus I2C (câblage, adresse, pull-ups) |
| `AHT20_ERR_CHECKSUM`      | CRC invalide (poly 0x31, init 0xFF)         |
| `AHT20_ERR_TIMEOUT`       | Timeout HAL dépassé                         |
| `AHT20_ERR_CALIBRATION`   | Bit CAL=0 après init — capteur non calibré  |
| `AHT20_ERR_BUSY`          | Mesure en cours (`async_busy = 1`)          |
| `AHT20_ERR_INVALID_PARAM` | Pointeur NULL ou handle non initialisé      |

### Diagnostic avancé (`AHT20_Handle`)

```c
dev.consecutive_errors  // incrémenté à chaque erreur, remis à 0 sur succès
dev.last_hal_error      // dernier code HAL_StatusTypeDef retourné par le bus
```

Quand `consecutive_errors >= AHT20_MAX_CONSECUTIVE_ERRORS` (défaut : 3), l'application peut décider un `SoftReset` ou un cycle `DeInit` / `Init`.

---

## Exemples

| Fichier                                  | Mode    | Description                                            |
| ---------------------------------------- | ------- | ------------------------------------------------------ |
| `exemple_aht20_polling.c`                | Polling | Lecture basique toutes les 2 s                         |
| `exemple_aht20_polling_error_handling.c` | Polling | Gestion erreurs : retry + `consecutive_errors` + stats |
| `exemple_aht20_polling_reset.c`          | Polling | `SoftReset()` + monitoring registre statut             |
| `exemple_aht20_polling_features.c`       | Polling | Humidité brute + index VOC + seuils                    |
| `exemple_aht20_async_it.c`               | Async   | IT avec `TriggerEvery`, callbacks IRQ-safe, `IsIdle`   |

### Matrice couverture API

| API                           | polling | err_handling | reset | features | async_it |
| ----------------------------- | :-----: | :----------: | :---: | :------: | :------: |
| `AHT20_Init`                  |    x    |      x       |   x   |    x     |    x     |
| `AHT20_DeInit`                |         |      x       |   x   |          |    x     |
| `AHT20_SoftReset`             |         |      x       |   x   |          |          |
| `AHT20_ReadMeasurements`      |    x    |      x       |   x   |    x     |          |
| `AHT20_GetStatus`             |         |              |   x   |          |          |
| `AHT20_StatusToString`        |    x    |      x       |   x   |    x     |    x     |
| `AHT20_Async_Init`            |         |              |       |          |    x     |
| `AHT20_Async_SetCallbacks`    |         |              |       |          |    x     |
| `AHT20_Async_SetIrqCallbacks` |         |              |       |          |    x     |
| `AHT20_ReadAll_IT`            |         |              |       |          |    x     |
| `AHT20_Async_TriggerEvery`    |         |              |       |          |    x     |
| `AHT20_Async_Process`         |         |              |       |          |    x     |
| `AHT20_Async_IsIdle`          |         |              |       |          |    x     |

---

## Matrice couverture datasheet

| Fonction datasheet                  | Statut       | Notes                                   |
| ----------------------------------- | ------------ | --------------------------------------- |
| Mesure température (20 bits)        | SUPPORTÉ     | `AHT20_ReadMeasurements`                |
| Mesure humidité (20 bits)           | SUPPORTÉ     | `AHT20_ReadMeasurements`                |
| CRC-8 (poly 0x31, init 0xFF)        | SUPPORTÉ     | Vérifié systématiquement                |
| Soft reset (`0xBA`)                 | SUPPORTÉ     | `AHT20_SoftReset`                       |
| Lecture registre statut             | SUPPORTÉ     | `AHT20_GetStatus` — bit CAL [3] vérifié |
| Init calibration (`0xBE 0x08 0x00`) | SUPPORTÉ     | Envoyé si CAL=0 dans `AHT20_Init`       |
| Mode IT asynchrone                  | SUPPORTÉ     | `AHT20_ReadAll_IT` + callbacks          |
| Adresse I2C configurable            | NON SUPPORTÉ | Fixe `0x38` (hardware AHT20)            |
| Mode DMA                            | NON SUPPORTÉ | N/A — trames ≤ 7 octets, gain nul       |
| Self-test intégré                   | N/A          | Absent de la datasheet AHT20            |

---

## Coexistence bus I2C

L'AHT20 (`0x38`) est conçu pour partager un bus I2C avec d'autres capteurs (SGP40 `0x59`, DHT20 `0x38` — **conflit** si présents simultanément).

```c
// I2C3 partagé : AHT20 + SGP40
AHT20_Init(&haht20, &hi2c3);
SGP40_Init(&hsgp40, &hi2c3);

// Dispatcher callbacks — chaque lib vérifie hi2c en interne
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    AHT20_Async_OnI2CMasterRxCplt(&aht20_ctx, hi2c);
    SGP40_Async_OnI2CMasterRxCplt(&sgp40_ctx, hi2c);
}
```

> Si `HAL_I2C_Master_*_IT` retourne `HAL_BUSY` (bus occupé), la lib repositionne `state = ASYNC_IDLE` sans incrémenter `consecutive_errors`. Le prochain `TriggerEvery` retentera.

---

## Paramètres configurables (`STM32_AHT20_conf.h`)

Toutes les macros sont protégées par `#ifndef` — surchargeables via `-D` ou avant l'include.

| Macro                              | Défaut | Unité | Description                                                      |
| ---------------------------------- | :----: | :---: | ---------------------------------------------------------------- |
| `AHT20_DEFAULT_TIMEOUT_MS`         | `100`  |  ms   | Timeout I2C pour les transactions bloquantes                     |
| `AHT20_WAIT_TIMEOUT_MS`            | `200`  |  ms   | Timeout watchdog d'attente mesure async                          |
| `AHT20_MAX_CONSECUTIVE_ERRORS`     |  `3`   |   —   | Seuil erreurs consécutives avant signalement critique            |
| `AHT20_DEFAULT_SAMPLE_INTERVAL_MS` | `2000` |  ms   | Intervalle minimal entre deux mesures async                      |
| `AHT20_DEBUG_ENABLE`               |   —    |   —   | Active `AHT20_StatusToString()` (laisser commenté en production) |

---

## Notes / Limitations

- **Adresse fixe `0x38`** : conflit avec DHT20 (même adresse) — ne pas placer les deux sur le même bus I2C.
- **Séquence Init** (datasheet §5.4.1) : lit le statut, envoie `0xBE 0x08 0x00` uniquement si bit CAL [3] = 0. En conditions normales (après power-on), CAL=1 d'emblée.
- **Délai mesure** : `AHT20_WAIT_TIMEOUT_MS = 200 ms` (datasheet spécifie ≥ 75 ms — marge ×2,5 pour bus chargé).
- **CRC** : poly `0x31` (x⁸ + x⁵ + x⁴ + 1), init `0xFF`. Vérifié sur chaque trame de mesure.
- **`GetStatus` pendant Init** : utilise `HAL_I2C_Master_Receive` direct (bypass du guard `initialized = false`).
- **DMA** : non implémenté — voir §Pré-requis CubeMX.

---

## Version

**v0.9.0** (2026-02-22)

- Conformité standard v0.9 : `consecutive_errors`, `last_hal_error`, `sample_interval_ms`, `DeInit`, `conf.h`, `StatusToString #ifdef`
- API asynchrone IT — `TriggerEvery` 3 params (ABI inter-lib)
- 5 exemples documentés

