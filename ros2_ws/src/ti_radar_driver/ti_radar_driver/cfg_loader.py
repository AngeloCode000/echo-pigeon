"""Sends a TI mmWave chirp configuration over the CLI UART.

The demo firmware echoes each command and replies 'Done' on success or an
'Error ...' line on failure. The port object only needs write(), readline()
and a timeout — tests pass a fake.

Pure module apart from time.sleep — must not import rclpy or serial.
"""

import time


class CfgError(RuntimeError):
    """A configuration command was rejected by the radar firmware."""


def parse_cfg_lines(text):
    """Strip comments ('%') and blank lines from a .cfg file's text."""
    lines = []
    for raw in text.splitlines():
        line = raw.split('%', 1)[0].strip()
        if line:
            lines.append(line)
    return lines


def send_command(port, command, inter_command_delay_s=0.02, max_reads=20):
    """Send one CLI command and read until 'Done' or an error line."""
    port.write((command + '\n').encode('ascii'))
    time.sleep(inter_command_delay_s)
    for _ in range(max_reads):
        reply = port.readline().decode('ascii', errors='replace').strip()
        if not reply:
            continue
        if reply == 'Done':
            return
        lower = reply.lower()
        if lower.startswith('error') or 'not recognized' in lower:
            raise CfgError(f"radar rejected '{command}': {reply}")
        # Anything else is the command echo or a banner line; keep reading.
    raise CfgError(f"no 'Done' response to '{command}'")


def send_cfg(port, cfg_text, skip_sensor_start=True):
    """Send a whole .cfg file.

    sensorStart is deferred to the caller by default so the data-port
    reader can be attached before the stream begins.
    """
    sent = []
    for command in parse_cfg_lines(cfg_text):
        if skip_sensor_start and command.startswith('sensorStart'):
            continue
        send_command(port, command)
        sent.append(command)
    return sent


def sensor_start(port):
    send_command(port, 'sensorStart')


def sensor_stop(port):
    send_command(port, 'sensorStop')
