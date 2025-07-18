#include <Arduino.h>
#include <rscp.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <driver/uart.h>
#include <driver/gpio.h>

// UART Configuration for RSCP Client Module
#define UART_CLIENT UART_NUM_2  // Client module connected to RX2 (GPIO16), TX2 (GPIO17)
#define ARM_PIN GPIO_NUM_2      // D2 pin for armed status indication
#define BUF_SIZE 256
#define BAUD_RATE 115200

// Buffers for protocol processing
static uint8_t uart_rx_buf[BUF_SIZE];
static uint8_t cobs_buf[BUF_SIZE];
static uint8_t tx_buf[BUF_SIZE];
static size_t raw_rx_idx = 0;  // Index for accumulating raw bytes

void setup() {
  // Setup USB Serial for monitoring
  Serial.begin(115200);
  
  // Configure ARM status pin - DEFAULT TO ARMED (HIGH)
  gpio_reset_pin(ARM_PIN);
  gpio_set_direction(ARM_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(ARM_PIN, 1);  // Start ARMED (HIGH)
  
  // Configure UART2 for bidirectional communication
  uart_config_t uart_config = {
    .baud_rate = BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0
  };
  
  uart_param_config(UART_CLIENT, &uart_config);
  uart_set_pin(UART_CLIENT, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_CLIENT, BUF_SIZE * 2, 0, 0, NULL, 0);
  
  Serial.println("RSCP ArmDisarm Decoder Ready");
  Serial.println("Initial state: ARMED (D2 HIGH)");
}

// COBS encode function compatible with Python cobs module
size_t cobs_encode(uint8_t *dst, const uint8_t *src, size_t len) {
  uint8_t *dst_start = dst;
  const uint8_t *end = src + len;
  uint8_t *code_ptr = dst++;
  uint8_t code = 0x01;

  while (src < end) {
    if (*src == 0) {
      *code_ptr = code;
      code_ptr = dst++;
      code = 0x01;
    } else {
      *dst++ = *src;
      if (++code == 0xFF) {
        *code_ptr = code;
        code_ptr = dst++;
        code = 0x01;
      }
    }
    src++;
  }
  
  *code_ptr = code;
  return dst - dst_start;
}

// COBS decode function compatible with Python cobs module
size_t cobs_decode(uint8_t *dst, const uint8_t *src, size_t len) {
  const uint8_t *src_end = src + len;
  uint8_t *dst_start = dst;
  
  while (src < src_end) {
    uint8_t code = *src++;
    for (uint8_t i = 1; i < code; i++) {
      *dst++ = *src++;
    }
    if (code < 0xFF && src < src_end) {
      *dst++ = 0;
    }
  }
  
  return dst - dst_start;
}

void send_acknowledgment() {
  // Create response envelope with acknowledgment
  rscp_ResponseEnvelope response = rscp_ResponseEnvelope_init_zero;
  response.which_response = rscp_ResponseEnvelope_acknowledge_tag;
  
  // Encode the response
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, rscp_ResponseEnvelope_fields, &response)) {
    Serial.printf("Ack encoding failed: %s\n", PB_GET_ERROR(&stream));
    return;
  }
  
  // COBS encode the acknowledgment
  uint8_t cobs_encoded[BUF_SIZE];
  size_t cobs_len = cobs_encode(cobs_encoded, tx_buf, stream.bytes_written);
  
  // Add frame delimiter (0x00) as Python client expects
  cobs_encoded[cobs_len++] = 0x00;
  
  // Send acknowledgment
  uart_write_bytes(UART_CLIENT, cobs_encoded, cobs_len);
  
  // Print sent acknowledgment for debugging
  Serial.print("Sent acknowledgment: ");
  for (size_t i = 0; i < cobs_len; i++) {
    Serial.printf("%02X ", cobs_encoded[i]);
  }
  Serial.println();
}

void handle_decoded_message(rscp_RequestEnvelope* envelope) {
  // Check if this is an ArmDisarm message
  if (envelope->which_request == rscp_RequestEnvelope_arm_disarm_tag) {
    if (envelope->request.arm_disarm.value) {
      Serial.println("Received ARM command - Rover is now armed (D2 HIGH)");
      gpio_set_level(ARM_PIN, 1);
    } else {
      Serial.println("Received DISARM command - Rover is now disarmed (D2 LOW)");
      gpio_set_level(ARM_PIN, 0);
    }
    // Send acknowledgment for ArmDisarm commands
    send_acknowledgment();
  } else if (envelope->which_request == rscp_RequestEnvelope_navigate_to_gps_tag) {
    Serial.println("Received NavigateToGPS command");
    send_acknowledgment();
  } else if (envelope->which_request == rscp_RequestEnvelope_set_stage_tag) {
    Serial.println("Received SetStage command");
    send_acknowledgment();
  } else {
    Serial.println("Received unknown command type");
  }
}

void process_frame(uint8_t* frame_data, size_t frame_len) {
  // COBS decode the frame
  uint8_t decoded_buf[BUF_SIZE];
  size_t decoded_len = cobs_decode(decoded_buf, frame_data, frame_len);
  
  Serial.printf("Decoded frame: %d bytes -> ", decoded_len);
  for (size_t i = 0; i < decoded_len; i++) {
    Serial.printf("%02X ", decoded_buf[i]);
  }
  Serial.println();
  
  // Try to decode as protobuf message
  pb_istream_t stream = pb_istream_from_buffer(decoded_buf, decoded_len);
  rscp_RequestEnvelope envelope = rscp_RequestEnvelope_init_zero;
  
  if (pb_decode(&stream, rscp_RequestEnvelope_fields, &envelope)) {
    Serial.println("Successfully decoded protobuf message");
    handle_decoded_message(&envelope);
  } else {
    Serial.printf("Protobuf decode failed: %s\n", PB_GET_ERROR(&stream));
  }
}

void loop() {
  // Read incoming data
  size_t len = uart_read_bytes(UART_CLIENT, uart_rx_buf, BUF_SIZE, 20 / portTICK_PERIOD_MS);
  
  if (len > 0) {
    Serial.printf("Received %d bytes: ", len);
    for (size_t i = 0; i < len; i++) {
      uint8_t byte = uart_rx_buf[i];
      Serial.printf("%02X ", byte);
      
      // Store byte in buffer
      if (raw_rx_idx < BUF_SIZE) {
        cobs_buf[raw_rx_idx++] = byte;
      }
      
      // Check for frame delimiter (0x00)
      if (byte == 0x00) {
        Serial.println("\nDetected frame delimiter");
        
        // Process frame if we have data
        if (raw_rx_idx > 1) {  // Minimum frame is 2 bytes (COBS code + delimiter)
          process_frame(cobs_buf, raw_rx_idx - 1);  // Exclude the 0x00 delimiter
        } else {
          Serial.println("Empty frame received");
        }
        
        // Reset buffer
        raw_rx_idx = 0;
      }
    }
    Serial.println();
  }
  
  // Handle buffer overflow
  if (raw_rx_idx >= BUF_SIZE) {
    Serial.println("Buffer overflow, resetting");
    raw_rx_idx = 0;
  }
  
  delay(10);
}