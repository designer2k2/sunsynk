"""Sunsync Modbus interface."""

import logging
import typing
from urllib.parse import urlparse

import attrs
from pymodbus import __version__ as version
from pymodbus.client import (
    AsyncModbusSerialClient,
    AsyncModbusTcpClient,
    AsyncModbusUdpClient,
    ModbusBaseClient,
)
from pymodbus.framer import FramerType

from sunsynk.sunsynk import Sunsynk

_LOGGER = logging.getLogger(__name__)


def _crc16(data: bytes) -> int:
    """Calculate Modbus RTU CRC16."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


@attrs.define
class PySunsynk(Sunsynk):
    """Sunsync Modbus class."""

    client: ModbusBaseClient = None  # type:ignore

    def _new_client(self) -> ModbusBaseClient:
        """Create a new client."""
        url = urlparse(f"{self.port}")
        if url.hostname:
            host, port = url.hostname, url.port or 502

            client: AsyncModbusTcpClient | AsyncModbusUdpClient | None = None

            match url.scheme:  # python 3.10 minimum
                case "serial-tcp":  # RTU-over-TCP
                    client = AsyncModbusTcpClient(
                        host=host, port=port, framer=FramerType.RTU
                    )
                case "tcp":
                    client = AsyncModbusTcpClient(host=host, port=port)
                case "serial-udp":  # RTU-over-UDP
                    client = AsyncModbusUdpClient(
                        host=host, port=port, framer=FramerType.RTU
                    )
                case "udp":
                    client = AsyncModbusUdpClient(host=host, port=port)
                case _:
                    raise NotImplementedError(
                        "Unknown scheme {url.scheme}: Only tcp and serial-tcp are supported"
                    )

            _LOGGER.info("PyModbus %s %s: %s:%s", version, url.scheme, host, port)
            return client

        _LOGGER.info("PyModbus %s Serial: %s", version, self.port)

        server_id = self.server_id

        def _patch_slave_id(sending: bool, data: bytes) -> bytes:
            """Patch slave ID 0 in responses to match server_id.

            Some Deye/Virtus inverter firmware always responds with slave ID 0
            regardless of the configured Modbus SN, causing pymodbus to reject
            the response. Only patches if the original CRC is valid, then
            recalculates CRC over the patched frame.
            Note: sending=False means we are receiving data from the inverter.
            """
            if not sending and len(data) > 4 and data[0] == 0:
                # Verify original CRC before patching
                body = data[:-2]
                orig_crc = (data[-2]) | (data[-1] << 8)
                if _crc16(body) == orig_crc:
                    # CRC valid - patch slave ID and recalculate CRC
                    patched_body = bytes([server_id]) + body[1:]
                    crc = _crc16(patched_body)
                    data = patched_body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
                else:
                    _LOGGER.debug("Slave ID 0 response with invalid CRC, not patching")
            return data

        return AsyncModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            # method="rtu",
            stopbits=1,
            bytesize=8,
            trace_packet=_patch_slave_id,
        )

    async def connect(self) -> None:
        """Connect. Will create a new client if required."""
        if not self.client:
            self.client = self._new_client()

        if not self.client.connected:
            await self.client.connect()

        if not self.client.connected:
            raise ConnectionError

    async def write_register(self, *, address: int, value: int) -> bool:
        """Write to a register - Sunsynk supports modbus function 0x10."""
        await self.connect()
        try:
            res = await self.client.write_registers(
                address=address,
                values=[value],  # type:ignore
                slave=self.server_id,
            )
            if res.function_code < 0x80:  # test that we are not an error
                return True
            _LOGGER.error("failed to write register %s=%s", address, value)
        except TimeoutError:
            _LOGGER.error("timeout writing register %s=%s", address, value)
        self.timeouts += 1
        return False

    async def read_holding_registers(
        self, start: int, length: int
    ) -> typing.Sequence[int]:
        """Read a holding register."""
        await self.connect()
        res = await self.client.read_holding_registers(  # type:ignore
            address=start, count=length, slave=self.server_id
        )
        if res.function_code >= 0x80:  # test that we are not an error
            raise OSError(
                f"failed to read register {start} - function code: {res.function_code}"
            )
        return res.registers
