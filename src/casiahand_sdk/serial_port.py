"""Resolve a Linux USB serial device without switching to another adapter."""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class UsbSerialDevice:
    port: str
    vendor: str
    product: str
    serial: str | None
    topology: str


def discover_usb_serial(sys_class_tty: Path = Path("/sys/class/tty")) -> list[UsbSerialDevice]:
    devices = []
    for tty in sorted(sys_class_tty.glob("tty*")):
        device = (tty / "device").resolve()
        for parent in (device, *device.parents):
            try:
                vendor = (parent / "idVendor").read_text().strip().lower()
                product = (parent / "idProduct").read_text().strip().lower()
            except OSError:
                continue
            try:
                serial = (parent / "serial").read_text().strip() or None
            except OSError:
                serial = None
            devices.append(UsbSerialDevice(f"/dev/{tty.name}", vendor, product, serial, str(parent)))
            break
    return devices


class SerialPortResolver:
    """Pin a unique USB serial identity, otherwise the original physical port."""

    def __init__(self, port_name: str):
        self.port_name = port_name
        self.identity: UsbSerialDevice | None = None
        self.use_serial = False

    def resolve(self) -> str:
        devices = discover_usb_serial()
        if self.identity is None:
            configured = str(Path(self.port_name).resolve())
            matches = [device for device in devices if device.port == configured]
            if not matches and not Path(self.port_name).exists() and not self.port_name.startswith("/dev/serial/"):
                matches = [
                    device
                    for device in devices
                    if device.vendor == "1a86" and Path(device.port).name.startswith(("ttyUSB", "ttyCH341USB"))
                ]
            if len(matches) != 1:
                raise ConnectionError(f"CASIA USB identity is unavailable or ambiguous for {self.port_name}")
            self.identity = matches[0]
            self.use_serial = (
                bool(self.identity.serial)
                and sum(
                    device.vendor == self.identity.vendor
                    and device.product == self.identity.product
                    and device.serial == self.identity.serial
                    for device in devices
                )
                == 1
            )
        pinned = self.identity
        matches = [
            device
            for device in devices
            if device.vendor == pinned.vendor
            and device.product == pinned.product
            and (device.serial == pinned.serial if self.use_serial else device.topology == pinned.topology)
        ]
        if len(matches) != 1:
            raise ConnectionError(f"CASIA original USB adapter is unavailable or ambiguous: {pinned.topology}")
        return matches[0].port
