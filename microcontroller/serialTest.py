import argparse
import os
import struct
import sys
import time

import serial
from serial import SerialException


MAGIC_BYTE = 0xA5
CRC_POLY = 0x1021
CRC_INIT = 0xFFFF
CRC_XOR_OUT = 0x0000
FRAME_HEADER_BYTES = 3
FRAME_CRC_BYTES = 2
ACCELERATION_T_ID = 1
ROTATION_T_ID = 2
ACCELERATION_PAYLOAD = struct.Struct("<fffQ")
ROTATION_PAYLOAD = struct.Struct("<fffffQ")
MESSAGE_PAYLOADS = {
    ACCELERATION_T_ID: ("acceleration_t", ACCELERATION_PAYLOAD),
    ROTATION_T_ID: ("rotation_t", ROTATION_PAYLOAD),
}


def open_port(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 1
    ser.write_timeout = 1
    ser.rtscts = False
    ser.dsrdtr = False

    if os.name == "posix":
        ser.exclusive = True

    ser.open()
    ser.dtr = False
    ser.rts = False
    time.sleep(0.25)
    return ser


def crc16_ccitt_false(data):
    crc = CRC_INIT

    for value in data:
        crc ^= value << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc ^ CRC_XOR_OUT


def write_payload_hex_line(payload, output_file):
    if not payload:
        return

    line = payload.hex()
    print(line, flush=True)

    if output_file is not None:
        output_file.write(line + "\n")
        output_file.flush()


def report_unsynced_bytes(data):
    if not data:
        return

    print(
        f"unsynced_bytes,count={len(data)},hex={data.hex()}",
        file=sys.stderr,
        flush=True,
    )


def frame_crc_is_valid(frame):
    message_type = frame[1]
    payload_length = frame[2]
    received_crc = (frame[-2] << 8) | frame[-1]
    calculated_crc = crc16_ccitt_false(frame[:-2])

    if received_crc == calculated_crc:
        print(
            f"crc_ok,type={message_type},payload_len={payload_length},crc=0x{received_crc:04x}",
            file=sys.stderr,
            flush=True,
        )
        return True

    print(
        f"crc_fail,type={message_type},payload_len={payload_length},"
        f"received=0x{received_crc:04x},calculated=0x{calculated_crc:04x}",
        file=sys.stderr,
        flush=True,
    )
    return False


def payload_is_valid(message_type, payload):
    payload_definition = MESSAGE_PAYLOADS.get(message_type)

    if payload_definition is None:
        print(
            f"unknown_message_type,type={message_type},payload_len={len(payload)}",
            file=sys.stderr,
            flush=True,
        )
        return False

    message_name, payload_struct = payload_definition

    if len(payload) != payload_struct.size:
        print(
            f"invalid_payload_len,type={message_type},name={message_name},"
            f"expected={payload_struct.size},actual={len(payload)}",
            file=sys.stderr,
            flush=True,
        )
        return False

    payload_struct.unpack(payload)
    return True


def emit_complete_frames(buffer, output_file):
    while buffer:
        magic_index = buffer.find(bytes([MAGIC_BYTE]))

        if magic_index < 0:
            report_unsynced_bytes(bytes(buffer))
            buffer.clear()
            return

        if magic_index > 0:
            unsynced = bytes(buffer[:magic_index])
            report_unsynced_bytes(unsynced)
            del buffer[:magic_index]

        if len(buffer) < FRAME_HEADER_BYTES:
            return

        payload_length = buffer[2]
        frame_length = FRAME_HEADER_BYTES + payload_length + FRAME_CRC_BYTES

        if len(buffer) < frame_length:
            return

        frame = bytes(buffer[:frame_length])
        del buffer[:frame_length]
        message_type = frame[1]
        payload = frame[FRAME_HEADER_BYTES:-FRAME_CRC_BYTES]

        if not frame_crc_is_valid(frame):
            continue

        if not payload_is_valid(message_type, payload):
            continue

        write_payload_hex_line(payload, output_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-o", "--output")
    parser.add_argument("--chunk-size", type=int, default=64)
    args = parser.parse_args()

    output_file = None
    buffer = bytearray()

    if args.output is not None:
        output_file = open(args.output, "w", encoding="utf-8")

    try:
        while True:
            ser = None

            try:
                ser = open_port(args.port, args.baud)
                print(f"connected,{args.port},{args.baud}", file=sys.stderr, flush=True)

                while True:
                    data = ser.read(args.chunk_size)

                    if not data:
                        continue

                    buffer.extend(data)
                    emit_complete_frames(buffer, output_file)

            except SerialException as e:
                print(f"serial_error,{e}", file=sys.stderr, flush=True)

                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

                time.sleep(1)

            except KeyboardInterrupt:
                if buffer:
                    print(
                        f"partial_bytes,count={len(buffer)},hex={bytes(buffer).hex()}",
                        file=sys.stderr,
                        flush=True,
                    )

                if ser is not None:
                    ser.close()

                print("stopped", file=sys.stderr, flush=True)
                return

    finally:
        if output_file is not None:
            output_file.close()


if __name__ == "__main__":
    main()
