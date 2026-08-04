import pytest

from ti_radar_driver.cfg_loader import (
    CfgError,
    parse_cfg_lines,
    send_cfg,
    send_command,
)


class FakePort:
    """Echoes each command then replies per a scripted response map."""

    def __init__(self, responses=None):
        self.responses = responses or {}
        self.written = []
        self._pending = []

    def write(self, data):
        command = data.decode().strip()
        self.written.append(command)
        reply = self.responses.get(command, 'Done')
        self._pending = [command, reply]

    def readline(self):
        if self._pending:
            return (self._pending.pop(0) + '\n').encode()
        return b''


def test_parse_cfg_lines_strips_comments_and_blanks():
    text = '\n'.join([
        '% full-line comment',
        '',
        'sensorStop',
        'frameCfg 0 2 16 0 100 1 0  % trailing comment',
        '   ',
    ])
    assert parse_cfg_lines(text) == ['sensorStop',
                                     'frameCfg 0 2 16 0 100 1 0']


def test_send_command_accepts_done_after_echo():
    port = FakePort()
    send_command(port, 'sensorStop', inter_command_delay_s=0)
    assert port.written == ['sensorStop']


def test_send_command_raises_on_error():
    port = FakePort(responses={'badCmd 1': 'Error -1'})
    with pytest.raises(CfgError, match='badCmd'):
        send_command(port, 'badCmd 1', inter_command_delay_s=0)


def test_send_command_raises_when_no_done():
    port = FakePort()
    port.responses['silentCmd'] = ''  # echo then nothing
    with pytest.raises(CfgError, match='silentCmd'):
        send_command(port, 'silentCmd', inter_command_delay_s=0, max_reads=3)


def test_send_cfg_defers_sensor_start():
    text = 'sensorStop\nchannelCfg 15 7 0\nsensorStart\n'
    port = FakePort()
    sent = send_cfg(port, text)
    assert sent == ['sensorStop', 'channelCfg 15 7 0']
    assert 'sensorStart' not in port.written
