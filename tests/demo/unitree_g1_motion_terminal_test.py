#!/usr/bin/env python3
"""Exercise the real terminal loop with --dry-run only; no DDS or robot traffic."""
import os
import pty
import select
import signal
import subprocess
import sys
import termios
import time
import unittest

BINARY = os.path.abspath(sys.argv.pop(1))


class Session:
    def __enter__(self):
        self.master, self.slave = pty.openpty()
        self.saved = termios.tcgetattr(self.slave)
        self.output = b""
        self.cursor = 0
        self.process = subprocess.Popen(
            [BINARY, "--dry-run"], stdin=self.slave, stdout=self.slave,
            stderr=self.slave, start_new_session=True)
        self.expect("verify posture after every transition.")
        return self

    def pump(self, timeout=0.05):
        if select.select([self.master], [], [], timeout)[0]:
            self.output += os.read(self.master, 65536)

    def expect(self, text):
        expected = text.encode()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            position = self.output.find(expected, self.cursor)
            if position >= 0:
                self.cursor = position + len(expected)
                return
            self.pump()
        raise AssertionError("Missing {!r}:\n{}".format(text, self.output.decode(errors="replace")))

    def send(self, value):
        os.write(self.master, value.encode())

    def action(self, key):
        self.send(key)
        self.expect("return_code=0")

    def exited(self):
        code = self.process.wait(timeout=4)
        while select.select([self.master], [], [], 0)[0]:
            self.pump(0)
        if code != 0:
            raise AssertionError("Exit {}: {}".format(code, self.output.decode(errors="replace")))
        if termios.tcgetattr(self.slave) != self.saved:
            raise AssertionError("Terminal settings were not restored")

    def __exit__(self, *_):
        if self.process.poll() is None:
            self.process.kill()  # Simulation only; never used against hardware.
            self.process.wait(timeout=4)
        os.close(self.master)
        os.close(self.slave)


class TerminalTests(unittest.TestCase):
    def test_startup_and_quit_never_write(self):
        with Session() as session:
            session.send("q")
            session.exited()
            self.assertNotIn(b"DRY RUN action=", session.output)

    def test_paste_and_retired_keys_do_not_execute(self):
        with Session() as session:
            session.send("pu")
            session.expect("Buffered/pasted input discarded")
            session.send("?")
            session.expect("verify posture after every transition.")
            session.send("!12345ZPK")
            time.sleep(0.1)
            session.send("q")
            session.exited()
            self.assertNotIn(b"DRY RUN action=", session.output)

    def test_queued_quit_survives_input_discard(self):
        with Session() as session:
            session.send("w" * 600 + "q")
            session.exited()
            self.assertNotIn(b"DRY RUN action=", session.output)

    def test_signals_and_eof_stop_and_restore_terminal(self):
        for termination in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP,
                            signal.SIGQUIT, signal.SIGTSTP, None):
            with self.subTest(termination=termination), Session() as session:
                session.action("p")
                if termination is None:
                    session.send("\x04")
                else:
                    session.process.send_signal(termination)
                session.exited()
                self.assertIn(b"DRY RUN action=0 ", session.output)

    def test_full_sequence_one_pulse_and_interrupt(self):
        with Session() as session:
            for key in ("p", "u", "t", "b"):
                session.action(key)
                time.sleep(3.05)  # Real event-loop clock, not fake policy time.
            session.send("r")
            session.expect("Armed for ONE")
            session.send("w")
            session.expect("DRY RUN action=13 vx=0.100000")
            session.expect("Pulse ended or state became unavailable")
            session.expect("return_code=0")
            session.send("w")
            session.expect("Press r after visually confirming")
            session.send("r")
            session.expect("Armed for ONE")
            session.send("a")
            session.expect("DRY RUN action=13 vx=0.000000 vy=0.080000")
            session.process.send_signal(signal.SIGINT)
            session.exited()
            self.assertEqual(session.output.count(b"DRY RUN action=13 "), 2)
            self.assertGreaterEqual(session.output.count(b"DRY RUN action=0 "), 2)


if __name__ == "__main__":
    unittest.main()
