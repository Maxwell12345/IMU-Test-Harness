import argparse
import csv
import os
import re
# import sys
import time

import serial
from serial import SerialException


LA_RE = re.compile(r"^LA,(\d+),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$")
RV_RE = re.compile(r"^RV,(\d+),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$")


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


def parse_line(line):
    m = LA_RE.match(line)
    if m:
        return {
            "type": "LA",
            "host_time": time.time(),
            "imu_time_us": int(m.group(1)),
            "x": float(m.group(2)),
            "y": float(m.group(3)),
            "z": float(m.group(4)),
            "i": "",
            "j": "",
            "k": "",
            "real": "",
            "accuracy": "",
        }

    m = RV_RE.match(line)
    if m:
        return {
            "type": "RV",
            "host_time": time.time(),
            "imu_time_us": int(m.group(1)),
            "x": "",
            "y": "",
            "z": "",
            "i": float(m.group(2)),
            "j": float(m.group(3)),
            "k": float(m.group(4)),
            "real": float(m.group(5)),
            "accuracy": float(m.group(6)),
        }

    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-o", "--output", default="imu_log.csv")
    parser.add_argument("--show-bad", action="store_true")
    args = parser.parse_args()

    fieldnames = [
        "type",
        "host_time",
        "imu_time_us",
        "x",
        "y",
        "z",
        "i",
        "j",
        "k",
        "real",
        "accuracy",
    ]

    started = False

    with open(args.output, "wb") as f:
        # writer = csv.DictWriter(f, fieldnames=fieldnames)
        # writer.writeheader()

        while True:
            ser = None

            try:
                ser = open_port(args.port, args.baud)
                print(f"connected,{args.port},{args.baud}")

                while True:
                    raw = ser.readline()

                    if not raw:
                        continue

                    # line = raw.decode("utf-8", errors="ignore")

                    # if not line:
                    #     continue

                    # row = parse_line(line)

                    # if not started:
                    #     if line == "BEGIN_IMU_CSV":
                    #         started = True
                    #         print("started")
                    #         continue
                    #
                    #     if row is not None:
                    #         started = True
                    #         print("started-from-data")
                    #
                    #     else:
                    #         if args.show_bad:
                    #             print(f"skip,{line}")
                    #         continue
                    #
                    # if row is None:
                    #     if args.show_bad:
                    #         print(f"bad,{line}")
                    #     continue
                    #
                    # writer.writerow(row)
                    print(f"{raw}",flush=True)
                    f.write(raw)
                    f.flush()

            except SerialException as e:
                print(f"serial_error,{e}")
                started = False

                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

                time.sleep(1)

            except KeyboardInterrupt:
                if ser is not None:
                    ser.close()
                print("stopped")
                return


if __name__ == "__main__":
    main()