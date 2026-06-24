import argparse
import asyncio
import sqlite3
import time

import serial_asyncio


class ImuProtocol(asyncio.Protocol):
    def __init__(self, q):
        self.q = q
        self.buf = b""
        self.bad = 0
        self.dropped = 0
        self.rows = 0

    def data_received(self, data):
        self.buf += data
        lines = self.buf.split(b"\n")
        self.buf = lines.pop()

        host_ns = time.time_ns()

        for line in lines:
            line = line.strip()
            if not line:
                continue

            try:
                p = line.split(b",")

                if p[0] == b"LA" and len(p) == 5:
                    row = (
                        "LA",
                        host_ns,
                        int(p[1]),
                        float(p[2]),
                        float(p[3]),
                        float(p[4]),
                    )
                elif p[0] == b"RV" and len(p) == 7:
                    row = (
                        "RV",
                        host_ns,
                        int(p[1]),
                        float(p[2]),
                        float(p[3]),
                        float(p[4]),
                        float(p[5]),
                        float(p[6]),
                    )
                else:
                    self.bad += 1
                    continue

                self.q.put_nowait(row)
                self.rows += 1

            except asyncio.QueueFull:
                self.dropped += 1
            except Exception:
                self.bad += 1


def open_db(path):
    db = sqlite3.connect(path, isolation_level=None)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("PRAGMA synchronous=NORMAL")
    db.execute("PRAGMA temp_store=MEMORY")
    db.execute("PRAGMA cache_size=-65536")
    db.execute("PRAGMA locking_mode=EXCLUSIVE")

    db.execute("""
        CREATE TABLE IF NOT EXISTS linear_acc (
            id INTEGER PRIMARY KEY,
            host_ns INTEGER NOT NULL,
            dev_us INTEGER NOT NULL,
            x REAL NOT NULL,
            y REAL NOT NULL,
            z REAL NOT NULL
        )
    """)

    db.execute("""
        CREATE TABLE IF NOT EXISTS rotation_vec (
            id INTEGER PRIMARY KEY,
            host_ns INTEGER NOT NULL,
            dev_us INTEGER NOT NULL,
            i REAL NOT NULL,
            j REAL NOT NULL,
            k REAL NOT NULL,
            real REAL NOT NULL,
            accuracy REAL NOT NULL
        )
    """)

    return db


def flush(db, la, rv):
    if not la and not rv:
        return

    db.execute("BEGIN IMMEDIATE")

    if la:
        db.executemany(
            "INSERT INTO linear_acc(host_ns, dev_us, x, y, z) VALUES (?, ?, ?, ?, ?)",
            la,
        )
        la.clear()

    if rv:
        db.executemany(
            "INSERT INTO rotation_vec(host_ns, dev_us, i, j, k, real, accuracy) VALUES (?, ?, ?, ?, ?, ?, ?)",
            rv,
        )
        rv.clear()

    db.execute("COMMIT")


async def db_writer(q, db_path, batch_size, flush_ms):
    db = open_db(db_path)
    la = []
    rv = []
    flush_s = flush_ms / 1000.0
    next_flush = time.perf_counter() + flush_s

    try:
        while True:
            timeout = max(0.0, next_flush - time.perf_counter())

            try:
                row = await asyncio.wait_for(q.get(), timeout=timeout)
            except asyncio.TimeoutError:
                flush(db, la, rv)
                next_flush = time.perf_counter() + flush_s
                continue

            if row is None:
                break

            if row[0] == "LA":
                la.append(row[1:])
            else:
                rv.append(row[1:])

            if len(la) + len(rv) >= batch_size:
                flush(db, la, rv)
                next_flush = time.perf_counter() + flush_s

    finally:
        flush(db, la, rv)
        db.close()


async def stats(protocol, q):
    last_rows = 0

    while True:
        await asyncio.sleep(1)
        rows = protocol.rows
        rate = rows - last_rows
        last_rows = rows

        print(
            f"rate={rate}/s total={rows} queued={q.qsize()} bad={protocol.bad} dropped={protocol.dropped}",
            flush=True,
        )


async def run(args):
    q = asyncio.Queue(maxsize=args.queue)

    loop = asyncio.get_running_loop()

    transport, protocol = await serial_asyncio.create_serial_connection(
        loop,
        lambda: ImuProtocol(q),
        args.port,
        baudrate=args.baud,
    )

    writer = asyncio.create_task(db_writer(q, args.db, args.batch, args.flush_ms))
    stat = asyncio.create_task(stats(protocol, q))

    try:
        await asyncio.Future()
    finally:
        transport.close()
        stat.cancel()
        await q.put(None)
        await writer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--db", default="imu.sqlite")
    parser.add_argument("--batch", type=int, default=1000)
    parser.add_argument("--flush-ms", type=int, default=50)
    parser.add_argument("--queue", type=int, default=200000)
    args = parser.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()