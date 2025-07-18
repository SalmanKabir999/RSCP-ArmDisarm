# ESP32 RSCP ArmDisarm Controller

## Overview
This project implements a rover control system with:
- ESP32 firmware that decodes RSCP (Rover Serial Control Protocol) messages
- Python client script (`arm.py`) to send Arm/Disarm commands

## Features
### ESP32 Firmware
- 🛡️ Decodes RSCP protocol using nanopb
- 📡 Processes COBS-encoded frames
- 🔧 Controls GPIO2 (D2) pin state (HIGH=Armed, LOW=Disarmed)
- 🔄 Bidirectional communication with acknowledgments
- 🧪 Detailed serial debug output
- 🚦 Default armed state on startup

### Python Client
- 📤 Sends ArmDisarm commands via serial
- 🔍 Listens for acknowledgments
- ⏱️ 5-second timeout for ACKs
- 📝 Clear console output

## Hardware Setup
| ESP32 Pin | Connection        | Direction |
|-----------|-------------------|-----------|
| GPIO16    | Client Module TX  | Input     |
| GPIO17    | Client Module RX  | Output    |
| GPIO2     | Status LED/Relay  | Output    |
| GND       | Common Ground     | -         |

## Installation
### ESP32
1. Upload firmware to ESP32

### Python Client
```bash
pip3 install pyserial cobs protobuf
pip3 install https://github.com/anatolianroverchallenge/rscp/releases/latest/download/rscp_protobuf.zip
```

## Usage
### Running the Client
```bash
python arm.py
```

## Configuration If Needed***
### ESP32 Code
Modify these constants in the code:
```cpp
#define UART_CLIENT UART_NUM_2  // UART port
#define ARM_PIN GPIO_NUM_2      // Control pin (D2)
#define BUF_SIZE 256            // UART buffer size
#define BAUD_RATE 115200        // Communication speed
```

### Modifying Commands
Edit these lines in `arm.py`:
```python
print("\nsend -> ArmDisarm(True)") # ARM command (Line:60)
req.arm_disarm.value = True   # ARM command (Line:62)
```
```python
print("\nsend -> ArmDisarm(False)") # DISARM command (Line:60)
req.arm_disarm.value = False  # DISARM command (Line:62)
```

## Expected Output
### Python Client
```plaintext
send -> ArmDisarm(True)
received -> Acknowledge
```

### ESP32 Serial Monitor
```plaintext
RSCP ArmDisarm Decoder Ready
Initial state: ARMED (D2 HIGH)
Received 6 bytes: 05 0A 02 08 01 00 
Detected frame delimiter
Decoded frame: 4 bytes -> 0A 02 08 01 
Successfully decoded protobuf message
Received ARM command - Rover is now armed (D2 HIGH)
Sent acknowledgment: 02 0A 01 00 
```

## Troubleshooting
| Issue                      | Solution                      |
|----------------------------|-------------------------------|
| No acknowledgment          | Check wiring (TX2/RX2)       |
| Serial errors              | Verify port permissions      |
| Decoding failures          | Check baud rate (115200)     |
| Buffer issues              | Increase BUF_SIZE in firmware|
