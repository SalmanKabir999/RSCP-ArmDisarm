import serial
import time
from cobs import cobs
import rscp_protobuf

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def send(ser, req):
    encoded = cobs.encode(req.SerializeToString()) + b'\x00'
    ser.write(encoded)

def receive_until_ack(ser, timeout=5.0):
    buffer = b""
    start = time.time()
    got_ack = False
    messages = []

    while time.time() - start < timeout:
        if ser.in_waiting:
            byte = ser.read()
            if byte == b'\x00':
                try:
                    decoded = cobs.decode(buffer)
                    response = rscp_protobuf.ResponseEnvelope()
                    response.ParseFromString(decoded)
                    messages.append(response)
                except Exception:
                    pass
                buffer = b""
            else:
                buffer += byte

    for response in messages:
        fields = [f.name for f, _ in response.ListFields()]
        if "acknowledge" in fields:
            print("received -> Acknowledge")
            got_ack = True

    for response in messages:
        for field, _ in response.ListFields():
            if field.name == "message":
                print(f"received -> {response.message}")
            elif field.name == "gps_coordinate":
                gps = response.gps_coordinate
                print(f"received -> Coordinate({gps.latitude}, {gps.longitude}, {gps.altitude})")
            elif field.name == "distance":
                print(f"received -> distance: {response.distance:.2f} meters")
            elif field.name == "task_finished":
                print("received -> TaskCompleted")

    return got_ack

# ================================
# ========== MAIN CODE ==========
# ================================
if __name__ == "__main__":
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print("\nsend -> ArmDisarm(False)") ### Change it to ArmDisarm(Flase) / ArmDisarm(True) to send the Arm and Disarm msg respectively
            req = rscp_protobuf.RequestEnvelope()
            req.arm_disarm.value = False     ## Change it to True / Flase to send Arm and Disarm msg Respectively
            send(ser, req)
            ack = receive_until_ack(ser)

            if not ack:
                print("No ACK received.")
            else:
                print("Mission step completed successfully.")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

