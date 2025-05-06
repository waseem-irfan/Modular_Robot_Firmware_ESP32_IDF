#include "dw3000.h"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "wifi_connect.h"
#include "nvs_flash.h"
#include "mpu6050_rpy.h"
#include "mqtt_comm.h"
#include "unity.h"
#include "servo_motor.h"
// #include "driver/gpio.h"
}
SemaphoreHandle_t mutex;
QueueHandle_t sensor_data_queue;
const char *MPU_TAG = "mpu6050_orientation";
#define APP_NAME "SS TWR INIT v1.0"

#define SERVO_PIN1 GPIO_NUM_12
#define SERVO_PIN2 GPIO_NUM_27
#define CHANNEL_1 LEDC_CHANNEL_0
#define CHANNEL_2 LEDC_CHANNEL_1

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

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 10

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16397
#define RX_ANT_DLY 16397

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TIMEOUT_UUS 1000

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;
// enum to identify data of sensor
typedef enum{
  DATA_TYPE_MPU6050,
  DATA_TYPE_UWB
}sensor_type_t;
// unified structure for queue handling
typedef struct{
  sensor_type_t type;
  union {
    struct{
        float roll;
        float pitch;
        float yaw;
    }mpu;
    struct{
        double distance;
    }uwb;
  };
}sensor_data_t;

// === TASK: MPU6050 Orientation Task ===
void mpu6050_rpy_task(void *params)
{
    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    int64_t last_time = esp_timer_get_time();  // μs

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Wrong WHO_AM_I");

    while (1)
    {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)))  // longer window for MPU read
        {
          printf("mutex taken by mpu6050_task\n");
            int64_t now = esp_timer_get_time();  // in μs
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
            if (yaw >= 360.0f) yaw -= 360.0f;
            if (yaw < 0.0f) yaw += 360.0f;

            ESP_LOGI(MPU_TAG, "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, dt=%.3fs", roll, pitch, yaw, dt);

            // sending it to queue
            sensor_data_t data;
            data.type = DATA_TYPE_MPU6050;
            data.mpu.roll = roll;
            data.mpu.pitch = pitch;
            data.mpu.yaw = yaw;
            if(xQueueSend(sensor_data_queue, &data, pdMS_TO_TICKS(20))){
              printf("MPU6050 data sent to queue\n");
            }
            else{
              printf("failed to send mpu6050 data to queue\n");
            }
            xSemaphoreGive(mutex);
            printf("mutex releassed by mpu6050_task\n");
        }
        else
        {
            ESP_LOGW(MPU_TAG, "MPU6050 reading timed out");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // ~10 Hz
    }

    mpu6050_delete(mpu6050);
    i2c_driver_delete(I2C_MASTER_NUM);
    vTaskDelete(NULL);
}


// === TASK: Servo Control ===
void servo_motor_task(void * param)
{
    while (1)
    {
      set_servo_speed(15, CHANNEL_1);
      set_servo_speed(-15, CHANNEL_2);
      vTaskDelay(pdMS_TO_TICKS(10));  // ~50 Hz
    }
}
// Creating a task for UWB reading
void UWB_task(void *params)
{
  while (1)
  {
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(200)))
    {
      printf("mutex tasken by uwb_task\n");
      tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

      /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
       * set by dwt_setrxaftertxdelay() has elapsed. */

      dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

      /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
      {
      }

      /* Increment frame sequence number after transmission of the poll message (modulo 256). */
      frame_seq_nb++;
      // if (status_reg & SYS_STATUS_RXFCG_BIT_MASK){
      //   printf("RXFCG = 1\n");
      // }
      // else{
      //   printf("RXFCG = 0\n");
      // }
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

          /* Check that the frame is the expected response from the companion "SS TWR responder" example.
           * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
          rx_buffer[ALL_MSG_SN_IDX] = 0;
          if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
          {
            uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32_t rtd_init, rtd_resp;
            float clockOffsetRatio;

            /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
            clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

            /* Get timestamps embedded in response message. */
            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;

            /* Display computed distance on LCD. */
            snprintf(dist_str, sizeof(dist_str), "DIS: %3.2f cm", distance * 100);
            test_run_info((unsigned char *)dist_str);

            sensor_data_t data;
            data.type = DATA_TYPE_UWB;
            data.uwb.distance = distance;
            if(xQueueSend(sensor_data_queue, &data, pdMS_TO_TICKS(50))){
              printf("UWB data sent to queue\n");
            }
            else{
              printf("failed to send UWB data to queue\n");
            }

          }
        }
      }
      else
      {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      }

      /* Execute a delay between ranging exchanges. */
      Sleep(RNG_DELAY_MS);
      xSemaphoreGive(mutex);
      printf("mutex released by uwb_task\n");
    }
    else
    {
      printf("UWB sensor reading timedout\n");
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// Task to send message
void test_send_messages(void *param)
{
  sensor_data_t received_data;
    while (true)
    {
      if(xSemaphoreTake(mutex, pdMS_TO_TICKS(100))){
        printf("mutex taken by MQTT_task\n");
          /************************************** MQTT_INITIALIZATION **************************/
        
        if(xQueueReceive(sensor_data_queue, &received_data, pdMS_TO_TICKS(100))){
          char payload[128];
          switch(received_data.type){
            case DATA_TYPE_MPU6050:
              snprintf(payload, sizeof(payload), "roll: %.2f , pitch: %.2f, yaw: %.2f", received_data.mpu.roll,
                                                        received_data.mpu.pitch, received_data.mpu.yaw);
              mqtt_send("masterBOT/mpu6050", payload);
              break;
            
            case DATA_TYPE_UWB:
              snprintf(payload, sizeof(payload), "Distance : %.2f", received_data.uwb.distance);
              mqtt_send("masterBOT/uwb", payload);
              break;
          }

        }
        else{
          printf("failed to send sensors data to broker\n");
        }
        xSemaphoreGive(mutex);
        printf("mutex released by MQTT_task\n");
      }
      else{
        printf("MQTT writing timedout\n");
      }

      vTaskDelay(pdMS_TO_TICKS(120));
    }
}

extern "C"
{
  void app_main(void);
}

void app_main(void)
{
  
  // setting PWM for servos
  setup_pwm(SERVO_PIN1, CHANNEL_1);
  setup_pwm(SERVO_PIN2, CHANNEL_2);
  // mutex = xSemaphoreCreateMutex(); // create mutex
  // sensor_data_queue = xQueueCreate(20, sizeof(sensor_data_t)); // creating queue
  // /*************************************** WIFI INITIALIZATION *******************************/

  ESP_ERROR_CHECK(nvs_flash_init());
  // wifi_connect_init();
  // ESP_ERROR_CHECK(wifi_connect_sta("Signum Signal", "ntgl5273", 10000));
  // vTaskDelay(pdMS_TO_TICKS(1000));

  // /*************************************** MPU6050 INITIALIZATION ***************************/
  // mpu6050_init();

  // /************************************** UWB INITIALIZATION ***************************/

  // UART_init();
  // test_run_info((unsigned char *)APP_NAME);
  // /* Configure SPI rate, DW3000 supports up to 38 MHz */
  // /* Reset DW IC */
  // spiBegin(PIN_IRQ, PIN_RST);
  // spiSelect(PIN_SS);

  // delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  // while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  // {
  //   UART_puts("IDLE FAILED\r\n");
  //   while (1)
  //     ;
  // }

  // if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  // {
  //   UART_puts("INIT FAILED\r\n");
  //   while (1)
  //     ;
  // }

  // // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  // dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // /* Configure DW IC. See NOTE 6 below. */
  // if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  // {
  //   UART_puts("CONFIG FAILED\r\n");
  //   while (1)
  //     ;
  // }

  // /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  // dwt_configuretxrf(&txconfig_options);

  // /* Apply default antenna delay value. See NOTE 2 below. */
  // dwt_setrxantennadelay(RX_ANT_DLY);
  // dwt_settxantennadelay(TX_ANT_DLY);

  // /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
  //  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  // dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
  //  * Note, in real low power applications the LEDs should not be used. */
  // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  // mqtt_start();


  vTaskDelay(pdMS_TO_TICKS(10000));

  // All tasks
  // xTaskCreatePinnedToCore(UWB_task, "UWB Task", 1024 * 2, NULL, 5, NULL, 1);
  // xTaskCreatePinnedToCore(mpu6050_rpy_task, "MPU6050 Task", 1024*4, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(servo_motor_task, "Servo Task", 1024 * 2, NULL, 5, NULL, 0);
  // xTaskCreatePinnedToCore(test_send_messages, "MQTT Task", 1024 * 4, NULL, 3, NULL, 0);

}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 11.
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
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate). For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
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
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200 µs).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 11. The use of the clock offset value to correct the TOF calculation, significantly improves the result of the SS-TWR where the remote
 *     responder unit's clock is a number of PPM offset from the local initiator unit's clock.
 *     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delay is calibrated and set correctly.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/