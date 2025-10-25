#ifndef TUSB_OS_CUSTOM_H_
#define TUSB_OS_CUSTOM_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "tx_api.h"
#include "stm32g4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OSAL_TIMEOUT_WAIT_FOREVER
#define OSAL_TIMEOUT_WAIT_FOREVER 0xFFFFFFFFu
#endif

#ifndef OSAL_TIMEOUT_NOTIMEOUT
#define OSAL_TIMEOUT_NOTIMEOUT 0u
#endif

static inline ULONG _tusb_ms_to_ticks(uint32_t msec) {
  if (msec == OSAL_TIMEOUT_WAIT_FOREVER) {
    return TX_WAIT_FOREVER;
  }

  if ((msec == OSAL_TIMEOUT_NOTIMEOUT) || (msec == 0u)) {
    return TX_NO_WAIT;
  }

  uint64_t ticks = ((uint64_t) msec * TX_TIMER_TICKS_PER_SECOND + 999u) / 1000u;
  if (ticks == 0u) {
    ticks = 1u;
  }
  return (ULONG) ticks;
}

static inline void osal_task_delay(uint32_t msec) {
  ULONG ticks = _tusb_ms_to_ticks(msec);
  if (ticks == TX_NO_WAIT) {
    tx_thread_relinquish();
  } else {
    tx_thread_sleep(ticks);
  }
}

// --------------------------------------------------------------------
// Spinlock API
// --------------------------------------------------------------------
typedef UINT osal_spinlock_t;

#define OSAL_SPINLOCK_DEF(_name, _int_set) \
  osal_spinlock_t _name

static inline void osal_spin_init(osal_spinlock_t *ctx) {
  (void) ctx;
}

static inline void osal_spin_lock(osal_spinlock_t *ctx, bool in_isr) {
  (void) in_isr;
  *ctx = __get_PRIMASK();
  __disable_irq();
}

static inline void osal_spin_unlock(osal_spinlock_t *ctx, bool in_isr) {
  (void) in_isr;
  __set_PRIMASK(*ctx);
}

// --------------------------------------------------------------------
// Semaphore API
// --------------------------------------------------------------------
typedef TX_SEMAPHORE osal_semaphore_def_t;
typedef TX_SEMAPHORE *osal_semaphore_t;

static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t *semdef) {
  if (tx_semaphore_create(semdef, (CHAR *) "tusb_sem", 0u) != TX_SUCCESS) {
    return NULL;
  }
  return semdef;
}

static inline bool osal_semaphore_delete(osal_semaphore_t semd_hdl) {
  return tx_semaphore_delete(semd_hdl) == TX_SUCCESS;
}

static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr) {
  (void) in_isr;
  return tx_semaphore_put(sem_hdl) == TX_SUCCESS;
}

static inline bool osal_semaphore_wait(osal_semaphore_t sem_hdl, uint32_t msec) {
  return tx_semaphore_get(sem_hdl, _tusb_ms_to_ticks(msec)) == TX_SUCCESS;
}

static inline void osal_semaphore_reset(osal_semaphore_t const sem_hdl) {
  while (tx_semaphore_get(sem_hdl, TX_NO_WAIT) == TX_SUCCESS) {
    // drain pending counts
  }
}

// --------------------------------------------------------------------
// Mutex API
// --------------------------------------------------------------------
typedef TX_MUTEX osal_mutex_def_t;
typedef TX_MUTEX *osal_mutex_t;

static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t *mdef) {
  if (tx_mutex_create(mdef, (CHAR *) "tusb_mtx", TX_INHERIT) != TX_SUCCESS) {
    return NULL;
  }
  return mdef;
}

static inline bool osal_mutex_delete(osal_mutex_t mutex_hdl) {
  return tx_mutex_delete(mutex_hdl) == TX_SUCCESS;
}

static inline bool osal_mutex_lock(osal_mutex_t mutex_hdl, uint32_t msec) {
  return tx_mutex_get(mutex_hdl, _tusb_ms_to_ticks(msec)) == TX_SUCCESS;
}

static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl) {
  return tx_mutex_put(mutex_hdl) == TX_SUCCESS;
}

// --------------------------------------------------------------------
// Queue API
// --------------------------------------------------------------------
typedef struct {
  TX_QUEUE queue;
  ULONG *storage;
  ULONG item_words;
  ULONG length;
  const CHAR *name;
} osal_queue_def_t;

typedef osal_queue_def_t *osal_queue_t;

#define OSAL_QUEUE_DEF(_int_set, _name, _depth, _type)                                           \
  static ULONG _name##_storage[(_depth) * ((sizeof(_type) + sizeof(ULONG) - 1u) / sizeof(ULONG))]; \
  osal_queue_def_t _name = {                                                                     \
      .storage = _name##_storage,                                                                \
      .item_words = (ULONG) ((sizeof(_type) + sizeof(ULONG) - 1u) / sizeof(ULONG)),             \
      .length = (_depth),                                                                       \
      .name = (const CHAR *) #_name                                                             \
  }

static inline osal_queue_t osal_queue_create(osal_queue_def_t *qdef) {
  ULONG queue_bytes = qdef->item_words * qdef->length * sizeof(ULONG);
  if (tx_queue_create(&qdef->queue, (CHAR *) qdef->name, qdef->item_words, qdef->storage, queue_bytes) != TX_SUCCESS) {
    return NULL;
  }
  return qdef;
}

static inline bool osal_queue_delete(osal_queue_t qhdl) {
  return tx_queue_delete(&qhdl->queue) == TX_SUCCESS;
}

static inline bool osal_queue_receive(osal_queue_t qhdl, void *data, uint32_t msec) {
  return tx_queue_receive(&qhdl->queue, data, _tusb_ms_to_ticks(msec)) == TX_SUCCESS;
}

static inline bool osal_queue_send(osal_queue_t qhdl, void const *data, bool in_isr) {
  (void) in_isr;
  return tx_queue_send(&qhdl->queue, (void *) data, TX_NO_WAIT) == TX_SUCCESS;
}

static inline bool osal_queue_empty(osal_queue_t qhdl) {
  ULONG enqueued = 0u;
  tx_queue_info_get(&qhdl->queue, TX_NULL, &enqueued, TX_NULL, TX_NULL, TX_NULL, TX_NULL);
  return enqueued == 0u;
}

#ifdef __cplusplus
}
#endif

#endif
