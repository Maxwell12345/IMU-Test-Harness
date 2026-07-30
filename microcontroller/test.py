#!/usr/bin/env python3

import argparse
import csv
import queue
import serial
import sqlite3
import sys
import threading
import time
from serial.tools import list_ports

DATABASE_STOP = object()
STOP_EVENT = threading.Event()
printTimeGps = 0

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-d", "--database", default="imu_data.db")
    parser.add_argument("--queue-size", type=int, default=10000)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--flush-interval", type=float, default=0.25)
    parser.add_argument("--gps-baud", type=int, default=115200)
    return parser.parse_args()


def parse_line(line):
    try:
        fields = next(csv.reader([line]))
    except csv.Error:
        return None

    try:
        if fields[0] == "DRPY" and len(fields) == 5:
            return {
                "type": "DRPY",
                "timestamp_us": int(fields[1]),
                "d_roll": float(fields[2]),
                "d_pitch": float(fields[3]),
                "d_yaw": float(fields[4]),
            }

        if fields[0] == "RV" and len(fields) == 7:
            return {
                "type": "RV",
                "timestamp_us": int(fields[1]),
                "i": float(fields[2]),
                "j": float(fields[3]),
                "k": float(fields[4]),
                "real": float(fields[5]),
                "accuracy": float(fields[6]),
            }

        if fields[0] == "LA" and len(fields) == 5:
            return {
                "type": "LA",
                "timestamp_us": int(fields[1]),
                "ax": float(fields[2]),
                "ay": float(fields[3]),
                "az": float(fields[4]),
            }
    except (ValueError, IndexError):
        return None

    return None


def print_measurement(data):
    if data["type"] == "DRPY":
        print(
            f'DRPY  t={data["timestamp_us"]} us  '
            f'd_roll={data["d_roll"]:.6f} rad/s  '
            f'd_pitch={data["d_pitch"]:.6f} rad/s  '
            f'd_yaw={data["d_yaw"]:.6f} rad/s'
        )
        return

    if data["type"] == "RV":
        print(
            f'RV    t={data["timestamp_us"]} us  '
            f'i={data["i"]:.6f}  '
            f'j={data["j"]:.6f}  '
            f'k={data["k"]:.6f}  '
            f'real={data["real"]:.6f}  '
            f'accuracy={data["accuracy"]:.6f}'
        )
        return

    if data["type"] == "LA":
        print(
            f'LA    t={data["timestamp_us"]} us  '
            f'ax={data["ax"]:.6f} m/s²  '
            f'ay={data["ay"]:.6f} m/s²  '
            f'az={data["az"]:.6f} m/s²'
        )


def initialize_database(connection):
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=NORMAL")
    connection.execute("PRAGMA temp_store=MEMORY")
    connection.execute("PRAGMA busy_timeout=30000")

    connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS imu_delta_rotation_rate (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_us INTEGER NOT NULL,
            host_timestamp_ns INTEGER NOT NULL,
            d_roll REAL NOT NULL,
            d_pitch REAL NOT NULL,
            d_yaw REAL NOT NULL
        );

        CREATE TABLE IF NOT EXISTS imu_rotation_vector (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_us INTEGER NOT NULL,
            host_timestamp_ns INTEGER NOT NULL,
            i REAL NOT NULL,
            j REAL NOT NULL,
            k REAL NOT NULL,
            real REAL NOT NULL,
            accuracy REAL NOT NULL
        );

        CREATE TABLE IF NOT EXISTS imu_linear_acceleration (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp_us INTEGER NOT NULL,
            host_timestamp_ns INTEGER NOT NULL,
            ax REAL NOT NULL,
            ay REAL NOT NULL,
            az REAL NOT NULL
        );

        CREATE TABLE IF NOT EXISTS gps1_nmea (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            host_timestamp_ns INTEGER NOT NULL,
            usb_vendor_id INTEGER NOT NULL,
            usb_product_id INTEGER NOT NULL,
            usb_serial_number TEXT,
            nmea TEXT NOT NULL
        );

        CREATE TABLE IF NOT EXISTS gps2_nmea (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            host_timestamp_ns INTEGER NOT NULL,
            usb_vendor_id INTEGER NOT NULL,
            usb_product_id INTEGER NOT NULL,
            usb_serial_number TEXT,
            nmea TEXT NOT NULL
        );

        CREATE INDEX IF NOT EXISTS idx_gps1_host_timestamp
            ON gps1_nmea(host_timestamp_ns);

        CREATE INDEX IF NOT EXISTS idx_gps2_host_timestamp
            ON gps2_nmea(host_timestamp_ns);

        CREATE INDEX IF NOT EXISTS idx_gps1_serial
            ON gps1_nmea(usb_serial_number);

        CREATE INDEX IF NOT EXISTS idx_gps2_serial
            ON gps2_nmea(usb_serial_number);

        CREATE INDEX IF NOT EXISTS idx_delta_rotation_timestamp
            ON imu_delta_rotation_rate(timestamp_us);

        CREATE INDEX IF NOT EXISTS idx_rotation_vector_timestamp
            ON imu_rotation_vector(timestamp_us);

        CREATE INDEX IF NOT EXISTS idx_linear_acceleration_timestamp
            ON imu_linear_acceleration(timestamp_us);
        """
    )

    connection.commit()


def insert_batch(connection, measurements):
    delta_rotation_rows = []
    rotation_vector_rows = []
    linear_acceleration_rows = []
    gps1_rows = []
    gps2_rows = []

    for data in measurements:
        if data["type"] == "DRPY":
            delta_rotation_rows.append(
                (
                    data["timestamp_us"],
                    data["host_timestamp_ns"],
                    data["d_roll"],
                    data["d_pitch"],
                    data["d_yaw"],
                )
            )
        elif data["type"] == "RV":
            rotation_vector_rows.append(
                (
                    data["timestamp_us"],
                    data["host_timestamp_ns"],
                    data["i"],
                    data["j"],
                    data["k"],
                    data["real"],
                    data["accuracy"],
                )
            )
        elif data["type"] == "LA":
            linear_acceleration_rows.append(
                (
                    data["timestamp_us"],
                    data["host_timestamp_ns"],
                    data["ax"],
                    data["ay"],
                    data["az"],
                )
            )
        elif data["type"] == "GPS1":
            gps1_rows.append(
                (
                    data["host_timestamp_ns"],
                    data["usb_vendor_id"],
                    data["usb_product_id"],
                    data["usb_serial_number"],
                    data["nmea"],
                )
            )

        elif data["type"] == "GPS2":
            gps2_rows.append(
                (
                    data["host_timestamp_ns"],
                    data["usb_vendor_id"],
                    data["usb_product_id"],
                    data["usb_serial_number"],
                    data["nmea"],
                )
            )

    with connection:
        if delta_rotation_rows:
            connection.executemany(
                """
                INSERT INTO imu_delta_rotation_rate (
                    timestamp_us,
                    host_timestamp_ns,
                    d_roll,
                    d_pitch,
                    d_yaw
                ) VALUES (?, ?, ?, ?, ?)
                """,
                delta_rotation_rows,
            )

        if rotation_vector_rows:
            connection.executemany(
                """
                INSERT INTO imu_rotation_vector (
                    timestamp_us,
                    host_timestamp_ns,
                    i,
                    j,
                    k,
                    real,
                    accuracy
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                rotation_vector_rows,
            )

        if linear_acceleration_rows:
            connection.executemany(
                """
                INSERT INTO imu_linear_acceleration (
                    timestamp_us,
                    host_timestamp_ns,
                    ax,
                    ay,
                    az
                ) VALUES (?, ?, ?, ?, ?)
                """,
                linear_acceleration_rows,
            )

        if gps1_rows:
            connection.executemany(
                """
                INSERT INTO gps1_nmea (
                    host_timestamp_ns,
                    usb_vendor_id,
                    usb_product_id,
                    usb_serial_number,
                    nmea
                ) VALUES (?, ?, ?, ?, ?)
                """,
                gps1_rows,
            )


        if gps2_rows:
            connection.executemany(
                """
                INSERT INTO gps2_nmea (
                    host_timestamp_ns,
                    usb_vendor_id,
                    usb_product_id,
                    usb_serial_number,
                    nmea
                ) VALUES (?, ?, ?, ?, ?)
                """,
                gps2_rows,
            )

def find_ublox_ports():
    """
    Returns a list of serial device paths corresponding to u-blox GPS receivers.
    """

    ports = []

    for port in list_ports.comports():
        manufacturer = (port.manufacturer or "").lower()
        description = (port.description or "").lower()
        product = (port.product or "").lower()
        hwid = (port.hwid or "").lower()

        if (
            "u-blox" in manufacturer
            or "u-blox" in description
            or "u-blox" in product
            or "1546" in hwid        # u-blox USB Vendor ID
        ):
            ports.append({
                "device": port.device,
                "vendor_id": port.vid or 0,
                "product_id": port.pid or 0,
                "serial_number": port.serial_number,
            })

    return sorted(ports, key=lambda x: x["device"])

def print_serial_devices():
    for port in list_ports.comports():
        print(
            f"{port.device:15} "
            f"manufacturer={port.manufacturer!r} "
            f"product={port.product!r} "
            f"description={port.description!r} "
            f"hwid={port.hwid}"
        )


class DatabaseWriter(threading.Thread):
    def __init__(self, database_path, measurement_queue, batch_size, flush_interval):
        super().__init__(name="database-writer")
        self.database_path = database_path
        self.measurement_queue = measurement_queue
        self.batch_size = batch_size
        self.flush_interval = flush_interval
        self.error = None
        self.rows_written = 0

    def run(self):
        connection = None

        try:
            connection = sqlite3.connect(self.database_path, timeout=30.0)
            initialize_database(connection)

            batch = []
            flush_deadline = time.monotonic() + self.flush_interval

            while True:
                timeout = max(0.0, flush_deadline - time.monotonic())

                try:
                    data = self.measurement_queue.get(timeout=timeout)
                except queue.Empty:
                    data = None

                if data is DATABASE_STOP:
                    if batch:
                        insert_batch(connection, batch)
                        self.rows_written += len(batch)
                    break

                if data is not None:
                    batch.append(data)

                if batch and (len(batch) >= self.batch_size or time.monotonic() >= flush_deadline):
                    insert_batch(connection, batch)
                    self.rows_written += len(batch)
                    batch.clear()
                    flush_deadline = time.monotonic() + self.flush_interval

        except Exception as error:
            self.error = error

        finally:
            if connection is not None:
                connection.close()


def stop_database_writer(writer, measurement_queue):
    while writer.is_alive():
        try:
            measurement_queue.put(DATABASE_STOP, timeout=0.25)
            break
        except queue.Full:
            if writer.error is not None or not writer.is_alive():
                break

    writer.join()

def gps_reader(
    port,
    receiver_name,
    measurement_queue,
    vendor_id,
    product_id,
    serial_number,
    baud,
):
    global printTimeGps
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            print(f"{receiver_name} connected on {port}")

            while not STOP_EVENT.is_set():
                raw = ser.readline()

                if not raw:
                    continue
                sentence = raw.decode(
                            "ascii",
                            errors="ignore"
                        ).strip()

                if printTimeGps < 10:
                    printTimeGps += 1
                else:
                    
                    printTimeGps = 0

                measurement_queue.put_nowait(
                    {
                        "type": receiver_name,
                        "host_timestamp_ns": time.monotonic_ns(),
                        "usb_vendor_id": vendor_id,
                        "usb_product_id": product_id,
                        "usb_serial_number": serial_number,
                        "nmea": sentence
                    },
                )

    except serial.SerialException as e:
        STOP_EVENT.set()
        print(
            f"{receiver_name} serial error: {e}",
            file=sys.stderr
        )


def main():
    args = parse_args()

    if args.queue_size <= 0:
        print("Queue size must be greater than zero", file=sys.stderr)
        return 1

    if args.batch_size <= 0:
        print("Batch size must be greater than zero", file=sys.stderr)
        return 1

    if args.flush_interval <= 0.0:
        print("Flush interval must be greater than zero", file=sys.stderr)
        return 1

    measurement_queue = queue.Queue(maxsize=args.queue_size)
    database_writer = DatabaseWriter(args.database, measurement_queue, args.batch_size, args.flush_interval)
    database_writer.start()

    gps_ports = find_ublox_ports()
    print("Detected u-blox receivers:")
    for gps in gps_ports:
        print(
            f"  {gps['device']} "
            f"VID={gps['vendor_id']} "
            f"PID={gps['product_id']} "
            f"SER={gps['serial_number']}"
        )

    if len(gps_ports) < 2:
        raise RuntimeError(
            f"Expected 2 u-blox receivers, found {len(gps_ports)}: {gps_ports}"
        )

    gps_threads = []

    for receiver_id, port in enumerate(gps_ports, start=1):

        receiver_name = f"GPS{receiver_id}"

        thread = threading.Thread(
            target=gps_reader,
            args=(
                port["device"],
                receiver_name,
                measurement_queue,
                port["vendor_id"],
                port["product_id"],
                port["serial_number"],
                args.gps_baud,
            ),
            daemon=True,
            name=receiver_name,
        )

        thread.start()
        gps_threads.append(thread)

        print(
            f"Started {receiver_name}: "
            f"{port['device']} "
            f"VID={port['vendor_id']} "
            f"PID={port['product_id']} "
            f"SER={port['serial_number']}"
        )

    exit_code = 0

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as serial_port:
            print(f"Connected to {args.port} at {args.baud} baud")
            print(f"Logging asynchronously to {args.database}")

            while True:
                if database_writer.error is not None:
                    raise RuntimeError(f"database writer failed: {database_writer.error}")

                raw = serial_port.readline()

                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                data = parse_line(line)

                if data is None:
                    continue

                data["host_timestamp_ns"] = time.monotonic_ns()

                try:
                    measurement_queue.put(data, timeout=1)
                except queue.Full:
                    raise RuntimeError(
                        f"database queue reached its {args.queue_size}-measurement limit"
                    )

                print_measurement(data)

    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        exit_code = 1

    except RuntimeError as error:
        print(f"Logging error: {error}", file=sys.stderr)
        exit_code = 1

    except KeyboardInterrupt:
        print("\nStopping")

    finally:
        STOP_EVENT.set()

        for thread in gps_threads:
            thread.join(timeout=2)

        stop_database_writer(database_writer, measurement_queue)

        if database_writer.error is not None:
            print(f"Database error: {database_writer.error}", file=sys.stderr)
            exit_code = 1
        else:
            print(f"Wrote {database_writer.rows_written} measurements to {args.database}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
