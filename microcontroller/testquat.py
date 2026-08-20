import argparse
import csv
import math
import serial
import sys


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    return parser.parse_args()


def parse_rotation_vector(line):
    try:
        fields = next(csv.reader([line]))

        if fields[0] != "RV" or len(fields) != 7:
            return None

        return {
            "timestamp_us": int(fields[1]),
            "i": float(fields[2]),
            "j": float(fields[3]),
            "k": float(fields[4]),
            "real": float(fields[5]),
            "accuracy": float(fields[6]),
        }
    except (csv.Error, ValueError, IndexError):
        return None


def get_enu(i, j, k, real):
    norm = math.sqrt(i * i + j * j + k * k + real * real)

    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("Invalid rotation-vector quaternion")

    x = i / norm
    y = j / norm
    z = k / norm
    w = real / norm

    yaw = math.atan2(
        2.0 * (x * y - w * z),
        1.0 - 2.0 * (x * x + z * z),
    )

    return real,i,j,k,math.degrees(yaw)


def main():
    args = parse_args()

    first_print = True
    
    ii = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as serial_port:
            print(f"Connected to {args.port} at {args.baud} baud")

            while True:
                raw = serial_port.readline()
                # print(raw)

                ii += 1
                if (ii % 10 != 0): continue
                ii = 0

                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                rotation = parse_rotation_vector(line)

                if rotation is None:
                    continue

                try:
                    w, i, j, k, heading = get_enu(
                        rotation["i"],
                        rotation["j"],
                        rotation["k"],
                        rotation["real"],
                    )
                except ValueError as error:
                    print(f"Quaternion error: {error}", file=sys.stderr)
                    continue

                if not first_print:
                    print("\033[4A", end="")

                print(f'\033[2Kt={rotation["timestamp_us"]} us')
                print(f'\033[2KWIJK={w:.3f} {i:.3f} {j:.3f} {k:.3f}')
                print(f'\033[2Kmag heading={heading:.3f}')
                print(
                    f'\033[2Kaccuracy='
                    f'{rotation["accuracy"] * 180.0 / math.pi:.6f}',
                    flush=True,
                )

                first_print = False

    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nStopping")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())