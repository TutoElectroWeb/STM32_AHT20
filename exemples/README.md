# Exemples STM32_AHT20

Exemples de référence couvrant **100 % de l'API publique** de `STM32_AHT20.h`.

Emplacement : `Drivers/STM32_AHT20/exemples/`

---

## Fichiers

| Fichier                                  | Objectif                                                              | Difficulté    |
| ---------------------------------------- | --------------------------------------------------------------------- | ------------- |
| `exemple_aht20_polling.c`                | Polling basique : init → mesure → affichage T/RH                      | Débutant      |
| `exemple_aht20_polling_error_handling.c` | Polling avancé : gestion erreurs consécutives + retry + statistiques  | Intermédiaire |
| `exemple_aht20_polling_reset.c`          | Polling spécialisé : `SoftReset()` + monitoring statut calibration    | Intermédiaire |
| `exemple_aht20_async_it_coverage.c`      | Coverage API async : callbacks IRQ-safe, GetData, ClearFlags, Reset   | Avancé        |
| `exemple_aht20_async_it.c`               | Mode IT app : `TriggerEvery` + `Tick` + recovery, compatible FreeRTOS | Avancé        |

> **Note IT vs DMA** : AHT20 implémente uniquement le mode **IT** (pas de DMA).
> Voir README lib §3 pour la décision technique : trames 7 octets, bus I2C partagé,
> aucun gain DMA mesurable sur ce capteur.

---

## Pré-requis CubeMX communs à tous les exemples

```
I2C3 (ou I2Cx) : Standard Mode 100 kHz, SDA/SCL avec pull-ups
NVIC I2C       : I2C event interrupt + I2C error interrupt activés (mode IT uniquement)
USART2         : Asynchronous, 115200 8N1 (printf → terminal série)
Exclure tous les autres exemple_xxx.c du build (un seul actif à la fois)
```

---

## Pattern harmonisé

Tous les exemples suivent le même schéma d'initialisation :

```c
/* PV — Variables globales */
AHT20_Handle_t haht20;

/* main() — USER CODE 2 */
AHT20_Status st = AHT20_Init(&haht20, &hi2c3);
if (st != AHT20_OK) {
    printf("ERREUR Init AHT20: %s\r\n", AHT20_StatusToString(st));
    Error_Handler();
}
```

`AHT20_StatusToString()` n'est disponible que si `AHT20_DEBUG_ENABLE` est
défini dans `STM32_AHT20_conf.h` (ou en tête de l'exemple).

---

## Matrice API → exemple

### Configuration et initialisation

| API               | polling | err_handling | reset | async_coverage | async_it |
| ----------------- | :-----: | :----------: | :---: | :------------: | :------: |
| `AHT20_Init`      |    x    |      x       |   x   |       x        |    x     |
| `AHT20_DeInit`    |    x    |      x       |   x   |       x        |    x     |
| `AHT20_GetStatus` |         |      x       |   x   |                |          |
| `AHT20_SoftReset` |         |              |   x   |                |          |

### Lecture mesures (polling synchrone)

| API                      | polling | err_handling | reset | async_coverage | async_it |
| ------------------------ | :-----: | :----------: | :---: | :------------: | :------: |
| `AHT20_ReadMeasurements` |    x    |      x       |   x   |                |          |

### API asynchrone I2C IT

| API                           | async_coverage | async_it |
| ----------------------------- | :------------: | :------: |
| `AHT20_Async_Init`            |       x        |    x     |
| `AHT20_Async_IsIdle`          |       x        |          |
| `AHT20_ReadAll_IT`            |       x        |    x     |
| `AHT20_Async_TriggerEvery`    |                |    x     |
| `AHT20_Async_Tick`            |       x        |    x     |
| `AHT20_Async_Process`         |       x        |          |
| `AHT20_Async_GetData`         |       x        |          |
| `AHT20_Async_HasData`         |       x        |          |
| `AHT20_Async_DataReadyFlag`   |       x        |          |
| `AHT20_Async_ErrorFlag`       |       x        |          |
| `AHT20_Async_Reset`           |       x        |          |
| `AHT20_Async_ClearFlags`      |       x        |          |
| `AHT20_Async_SetCallbacks`    |       x        |          |
| `AHT20_Async_SetIrqCallbacks` |       x        |          |
| Callbacks `OnI2CMasterTxCplt` |       x        |    x     |
| Callbacks `OnI2CMasterRxCplt` |       x        |    x     |
| Callbacks `OnI2CError`        |       x        |    x     |

> **Note nommage** : `exemple_aht20_async_it_coverage.c` couvre les fonctions async
> non présentes dans `async_it` (callbacks bas niveau, GetData, ClearFlags, Reset,
> SetIrqCallbacks). `exemple_aht20_async_it.c` est l'exemple applicatif recommandé
> (pattern `TriggerEvery`).

### Gestion des erreurs et statistiques

| API / Type                       | polling | err_handling | reset | async_coverage | async_it |
| -------------------------------- | :-----: | :----------: | :---: | :------------: | :------: |
| `AHT20_StatusToString`           |    x    |      x       |   x   |       x        |    x     |
| `AHT20_MAX_CONSECUTIVE_ERRORS`   |    x    |      x       |       |                |    x     |
| `AHT20_Stats_t` (stats internes) |         |      x       |       |                |          |

---

## Conventions

- Tous les exemples sont des templates CubeMX (`main.c`) avec zones `USER CODE BEGIN/END`
- Aucun `HAL_Delay` après `__disable_irq()` dans `Error_Handler`
- Gestion d'erreur via `AHT20_StatusToString()` (pas de magic numbers)
- `printf` redirigé vers UART2 via `__io_putchar()` dans tous les exemples
- Les exemples async sont **compatibles FreeRTOS** : `HAL_GetTick()` uniquement, pas de spin-wait
- `HAL_BUSY` retourné par `TriggerEvery` = FSM occupée ou intervalle non atteint (pas une erreur)

---

## Adaptation à votre projet

Pour un projet final, conservez la logique métier et adaptez :

1. Le périphérique I2C dans `AHT20_Init()` (ex: `&hi2c3` → `&hi2c1`)
2. L'instance UART printf dans `__io_putchar()` (ex: `&huart2` → `&huart3`)
3. L'intervalle de déclenchement async (`measure_interval_ms`) selon la fréquence souhaitée
4. La stratégie de recovery (`AHT20_MAX_CONSECUTIVE_ERRORS`) selon le contexte applicatif
