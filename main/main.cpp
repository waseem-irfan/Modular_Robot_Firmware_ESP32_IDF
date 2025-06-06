#include "dw3000.h"
#include "trilaterate.h"
extern "C"
{
#include "mqtt_comm.h"
#include "wifi_connect.h"
#include "nvs_flash.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "servo_motor.h"
#include "mpu6050_rpy.h"
#include "unity.h"
#include "esp_log.h"
}
QueueHandle_t distance_queue;
SemaphoreHandle_t mutex;
QueueHandle_t sensor_data_queue;

typedef enum
{
  DATA_TYPE_MPU6050,
  DATA_TYPE_UWB
} sensor_type_t;
// unified structure for queue handling
typedef struct
{
  sensor_type_t type;
  union
  {
    struct
    {
      float roll;
      float pitch;
      float yaw;
    } mpu;
    struct
    {
      float x;
      float y;
      float z;
    } uwb;
  };
} sensor_data_t;

#define MPU_TAG "mpu6050_rpy"

#define SERVO1_PIN GPIO_NUM_12
#define SERVO2_PIN GPIO_NUM_27
// #define MINI_SERVO_PIN GPIO_NUM_14

#define APP_NAME "SS TWR RESP v1.0"

// connection pins
const uint8_t PIN_RST = 26; // reset pin
const uint8_t PIN_IRQ = 4;  // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16397
#define RX_ANT_DLY 16397

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12 // Must be less than FRAME_LEN_MAX_EX
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#ifdef RPI_BUILD
#define POLL_RX_TO_RESP_TX_DLY_UUS 550
#endif // RPI_BUILD
#ifdef STM32F429xx
#define POLL_RX_TO_RESP_TX_DLY_UUS 450
#endif // STM32F429xx
#ifdef NRF52840_XXAA
#define POLL_RX_TO_RESP_TX_DLY_UUS 650
#endif // NRF52840_XXAA

#define POLL_RX_TO_RESP_TX_DLY_UUS 750

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;
/*
  Tasks Creation
*/
// Motor Task
void servo_motor_task(void *param)
{
  // set_servo_speed(+80, LEDC_CHANNEL_0); // left
  // set_servo_speed(-70, LEDC_CHANNEL_1); // right
  while (1)
  {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)))
    {
      printf("mutex taken by servo_task\n");
      printf("Running Two Servos\n");

      // Control code here (e.g., pwm set)
      // ...

      xSemaphoreGive(mutex);
      printf("mutex released by servo_task\n");
    }
    else
    {
      printf("Servo reading timed out\n");
    }

    vTaskDelay(pdMS_TO_TICKS(40)); // ~50 Hz
  }
}

// === TASK: MPU6050 Orientation Task ===
void mpu6050_rpy_task(void *params)
{
  esp_err_t ret;
  uint8_t mpu6050_deviceid;
  mpu6050_acce_value_t acce;
  mpu6050_gyro_value_t gyro;

  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  int64_t last_time = esp_timer_get_time(); // μs

  ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Wrong WHO_AM_I");

  while (1)
  {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200))) // longer window for MPU read
    {
      printf("mutex taken by mpu6050_task\n");
      int64_t now = esp_timer_get_time();        // in μs
      float dt = (now - last_time) / 1000000.0f; // seconds
      last_time = now;

      ret = mpu6050_get_acce(mpu6050, &acce);
      TEST_ASSERT_EQUAL(ESP_OK, ret);

      ret = mpu6050_get_gyro(mpu6050, &gyro);
      TEST_ASSERT_EQUAL(ESP_OK, ret);

      // Compute accelerometer-based Roll & Pitch
      float accel_roll = atan2f(acce.acce_y, acce.acce_z) * 180.0f / M_PI;
      float accel_pitch = atan2f(-acce.acce_x, sqrtf(acce.acce_y * acce.acce_y + acce.acce_z * acce.acce_z)) * 180.0f / M_PI;

      // Gyroscope rate
      float gyro_roll_rate = gyro.gyro_x;
      float gyro_pitch_rate = gyro.gyro_y;
      float gyro_yaw_rate = gyro.gyro_z;

      // Complementary filter
      roll = ALPHA * (roll + gyro_roll_rate * dt) + (1.0f - ALPHA) * accel_roll;
      pitch = ALPHA * (pitch + gyro_pitch_rate * dt) + (1.0f - ALPHA) * accel_pitch;

      // Yaw integration
      yaw += gyro_yaw_rate * dt;
      if (yaw >= 360.0f)
        yaw -= 360.0f;
      if (yaw < 0.0f)
        yaw += 360.0f;

      ESP_LOGI(MPU_TAG, "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, dt=%.3fs", roll, pitch, yaw, dt);

      // sending it to queue
      sensor_data_t data;
      data.type = DATA_TYPE_MPU6050;
      data.mpu.roll = roll;
      data.mpu.pitch = pitch;
      data.mpu.yaw = yaw;
      if (xQueueSend(sensor_data_queue, &data, pdMS_TO_TICKS(20)))
      {
        printf("MPU6050 data sent to queue\n");
      }
      else
      {
        printf("failed to send mpu6050 data to queue\n");
      }
      xSemaphoreGive(mutex);
      printf("mutex releassed by mpu6050_task\n");
    }
    else
    {
      ESP_LOGW(MPU_TAG, "MPU6050 reading timed out");
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
  }

  mpu6050_delete(mpu6050);
  i2c_driver_delete(I2C_MASTER_NUM);
  vTaskDelete(NULL);
}

// UWB_Task
void uwb_task(void *params)
{
  // TickType_t last_wake = xTaskGetTickCount();
  while (1)
  {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(500)))
    {
      printf("mutex taken by uwb_task\n");
      /* Activate reception immediately. */
      dwt_rxenable(DWT_START_RX_IMMEDIATE);

      /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
      {
      }

      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
      {
        uint32_t frame_len;

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer))
        {
          dwt_readrxdata(rx_buffer, frame_len, 0);

          /* Check that the frame is a poll sent by "SS TWR initiator" example.
           * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
          rx_buffer[ALL_MSG_SN_IDX] = 0;
          if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
          {
            uint32_t resp_tx_time;
            int ret;

            /* Retrieve poll reception timestamp. */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute response message transmission time. See NOTE 7 below. */
            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
            resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the final message. See NOTE 8 below. */
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /* Write and send the response message. See NOTE 9 below. */
            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED);
            printf("ret = %d\n", ret);
            /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
            if (ret == DWT_SUCCESS)
            {
              /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
              while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
              {
              };

              /* Clear TXFRS event. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

              /* Increment frame sequence number after transmission of the poll message (modulo 256). */
              frame_seq_nb++;

              // double distance;
              uwb_distances_t distances;
              Anchor A1 = {0.0, 0.0, 0.0};
              Anchor A2 = {100.0, 0.0, 0.0};
              Anchor A3 = {0.0, 100.0, 0.0};

              if (xQueueReceive(uwb_distances_queue, &distances, pdMS_TO_TICKS(50)))
              {
                printf("Anchor A1: %.2f cm\n", distances.A1);
                printf("Anchor A2: %.2f cm\n", distances.A2);
                printf("Anchor A3: %.2f cm\n", distances.A3);

                // Update measured distances
                A1.distance = distances.A1;
                A2.distance = distances.A2;
                A3.distance = distances.A3;

                Position result;
                if (trilateration(A1, A2, A3, result))
                {
                  printf("Robot Position X: %.2f cm, Y: %.2f cm, Z: %.2f cm\n", result.x, result.y, result.z);
                  sensor_data_t data;
                  data.type = DATA_TYPE_UWB;
                  data.uwb.x = result.x;
                  data.uwb.y = result.y;
                  data.uwb.z = result.z;

                  if (xQueueSend(sensor_data_queue, &data, pdMS_TO_TICKS(50)))
                  {
                    printf("UWB data sent to queue\n");

                  }
                  else
                  {
                    printf("Failed to send UWB to queue\n");
                  }
                }
                else
                {
                  printf("Trilateration failed — possible colinear anchors or bad distances.\n");
                }

                // vTaskDelay(pdMS_TO_TICKS(100)); // Optional delay
              }
              else
              {
                                          xSemaphoreGive(mutex);
                          printf("mutex released by uwb_task\n");
                printf("No queue Received\n");
              }
            }
          }
        }
      }
      else
      {
        /* Clear RX error events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      }

      // vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
      vTaskDelay(pdMS_TO_TICKS(40));
    }
  }
}

// Task to send message
void test_send_messages(void *params)
{
  sensor_data_t data;

  while (1)
  {
    if (xQueueReceive(sensor_data_queue, &data, portMAX_DELAY))
    {
      char json_payload[128];

      if (data.type == DATA_TYPE_UWB)
      {
        snprintf(json_payload, sizeof(json_payload),
                 "{\"type\": \"UWB\", \"x\": %.2f, \"y\": %.2f, \"z\": %.2f}",
                 data.uwb.x, data.uwb.y, data.uwb.z);
      }
      else if (data.type == DATA_TYPE_MPU6050)
      {
        snprintf(json_payload, sizeof(json_payload),
                 "{\"type\": \"MPU6050\", \"roll\": %.2f, \"pitch\": %.2f, \"yaw\": %.2f}",
                 data.mpu.roll, data.mpu.pitch, data.mpu.yaw);
      }

      mqtt_send("masterBOT/data", json_payload);
      printf("Published JSON: %s\n", json_payload);
    }
  }
}

extern "C"
{
  void app_main();
}

void app_main()
{
  uwb_distances_queue = xQueueCreate(5, sizeof(uwb_distances_t));
  mutex = xSemaphoreCreateMutex();                             // create mutex
  sensor_data_queue = xQueueCreate(20, sizeof(sensor_data_t)); // creating queue
  // MPU6050 INIT
  mpu6050_init();

  // Motor Initialization
  // setup_pwm(MINI_SERVO_PIN, LEDC_CHANNEL_2);
  setup_pwm(SERVO1_PIN, LEDC_CHANNEL_0);
  setup_pwm(SERVO2_PIN, LEDC_CHANNEL_1);
  // vTaskDelay(pdMS_TO_TICKS(20000));

  // WIFI
  nvs_flash_init();
  wifi_connect_init();
  wifi_connect_sta("Nalim", "0123456789", 10000);

  // MQTT
  mqtt_start();
  // delay(5000);
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // vTaskDelay(pdMS_TO_TICKS(30000));

  // All tasks
  xTaskCreatePinnedToCore(uwb_task, "UWB Task", 1024 * 2, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(mpu6050_rpy_task, "MPU6050 Task", 1024 * 4, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(servo_motor_task, "Servo Task", 1024 * 2, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(test_send_messages, "MQTT Task", 1024 * 4, NULL, 3, NULL, 0);
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (length of response frame)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 *
 *
 * 2. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 6. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW IC
 *    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *    8 bits.
 * 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *    time-of-flight computation) can be handled by a 32-bit subtraction.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 10. When running this example on the DW3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_06a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/