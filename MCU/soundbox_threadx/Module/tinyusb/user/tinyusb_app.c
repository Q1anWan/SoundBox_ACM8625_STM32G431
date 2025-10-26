#include "tinyusb_app.h"

#include <stdbool.h>
#include <stdint.h>

#include "audio_app.h"
#include "board_api.h"
#include "tusb.h"

#define TINYUSB_THREAD_TUD_STACK_SIZE 4096
#define TINYUSB_THREAD_APP_STACK_SIZE 4096
#define TINYUSB_THREAD_PRIORITY   5u

static TX_THREAD tinyusb_thread_tud;
static TX_THREAD tinyusb_thread_app;

static uint8_t tinyusb_thread_tud_stack[TINYUSB_THREAD_TUD_STACK_SIZE];
static uint8_t tinyusb_thread_app_stack[TINYUSB_THREAD_APP_STACK_SIZE];
static bool tinyusb_thread_created = false;

static void tinyusb_thread_tud_entry(ULONG argument);
static void tinyusb_thread_app_entry(ULONG argument);

UINT tinyusb_app_init(void) {
  if (tinyusb_thread_created) {
    return TX_SUCCESS;
  }

  UINT status = tx_thread_create(&tinyusb_thread_tud,
                                 (CHAR *) "TinyUSBTUD",
                                 tinyusb_thread_tud_entry,
                                 0,
                                 tinyusb_thread_tud_stack,
                                 sizeof(tinyusb_thread_tud_stack),
                                 TINYUSB_THREAD_PRIORITY,
                                 TINYUSB_THREAD_PRIORITY,
                                 TX_NO_TIME_SLICE,
                                 TX_AUTO_START);

  if (status == TX_SUCCESS) {
    tinyusb_thread_created = true;
  }

  return status;
}

static void tinyusb_thread_tud_entry(ULONG argument) {
  (void) argument;

  board_led_write(true);

  tusb_rhport_init_t dev_init = {
      .role = TUSB_ROLE_DEVICE,
      .speed = TUSB_SPEED_AUTO,
  };

  
  if (!tusb_init(BOARD_TUD_RHPORT, &dev_init)) {
    // Unable to initialise the USB device stack, block forever.
    while (1) {
      tx_thread_sleep(TX_WAIT_FOREVER);
    }
  }

  board_init_after_tusb();
  tinyusb_audio_app_init();

  tx_thread_create(&tinyusb_thread_app,
                                (CHAR *) "TinyUSBAPP",
                                tinyusb_thread_app_entry,
                                0,
                                tinyusb_thread_app_stack,
                                sizeof(tinyusb_thread_app_stack),
                                TINYUSB_THREAD_PRIORITY,
                                TINYUSB_THREAD_PRIORITY,
                                TX_NO_TIME_SLICE,
                                TX_AUTO_START);

  for (;;) {
    tud_task();
  }
}

static void tinyusb_thread_app_entry(ULONG argument) {
  (void) argument;

  for (;;) {
    tinyusb_audio_app_task();
    tx_thread_sleep(1);
  }
}
