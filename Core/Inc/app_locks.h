#ifndef APP_LOCKS_H
#define APP_LOCKS_H

#ifdef __cplusplus
extern "C" {
#endif

/* I2C2 bus lock (M1).
 * The ES8311 audio codec and the PCF85063 RTC share I2C2. Without
 * serialization a Mic_Start/Mic_Stop codec register sequence can interleave
 * with an AppTime_Sync() RTC read on the wire, corrupting the HAL_I2C state
 * machine / bus (same family as the F6 timestamp issue).
 *
 * Usage is sequence-level: hold the lock across a whole codec sequence or a
 * whole RTC read, never per single register. The lock is the innermost (leaf)
 * lock — no other mutex is taken while it is held, so it cannot deadlock.
 * Acquire/release are NULL-safe before the mutex is created (early boot).
 * The mutex itself lives in app_freertos.c next to the other RTOS mutexes. */
void AppI2c2Lock(void);
void AppI2c2Unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOCKS_H */
