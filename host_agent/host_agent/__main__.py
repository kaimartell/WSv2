import argparse

from host_agent.backends import list_backends, list_ble_devices, list_serial_devices
from host_agent.server import run_server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the workshop host agent server")
    parser.add_argument("--list", action="store_true", help="List serial + BLE devices and exit")
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host interface to bind (default 0.0.0.0 for Docker host.docker.internal access)",
    )
    parser.add_argument("--port", type=int, default=8000, help="Port to bind")
    parser.add_argument(
        "--backend",
        choices=list_backends() + ["real"],
        default="mock",
        help="Backend implementation",
    )
    parser.add_argument(
        "--hub-name",
        default="",
        help="SPIKE hub name filter for BLE scan/connect (optional)",
    )
    parser.add_argument("--hub-address", default="", help="BLE MAC/address of hub (optional)")
    parser.add_argument(
        "--serial-port",
        default="auto",
        help="USB serial port for spike_usb backend (default: auto)",
    )
    parser.add_argument("--baud", type=int, default=115200, help="USB serial baud rate for spike_usb")
    parser.add_argument(
        "--debug-repl-snippets",
        action="store_true",
        help="Include raw REPL snippet text in spike_usb errors/logs",
    )
    parser.add_argument("--motor-port", default="A", help="Motor port letter for spike_ble/spike_usb (A-F)")
    parser.add_argument("--scan-timeout", type=float, default=4.0, help="BLE scan timeout (seconds)")
    parser.add_argument("--connect-timeout", type=float, default=6.0, help="BLE connect timeout (seconds)")
    return parser.parse_args()


def _print_device_list(hub_name: str, timeout: float) -> None:
    print("Serial candidates:")
    serial_rows = list_serial_devices()
    if not serial_rows:
        print("  (none)")
    for row in serial_rows:
        device = row.get("device", "")
        description = row.get("description", "")
        hwid = row.get("hwid", "")
        print(f"  device={device} description={description} hwid={hwid}")

    print("\nBLE candidates:")
    ble_rows = list_ble_devices(timeout=timeout, name_filter=hub_name)
    if not ble_rows:
        print("  (none)")
        return
    for row in ble_rows:
        if "error" in row:
            print(f"  error: {row['error']}")
            continue
        print(
            f"  name={row.get('name', '') or '<unknown>'} "
            f"address={row.get('address', '')} rssi={row.get('rssi', '')}"
        )


def main() -> None:
    args = parse_args()
    if args.list:
        _print_device_list(hub_name=args.hub_name, timeout=args.scan_timeout)
        return

    backend_kwargs = {
        "hub_name": args.hub_name,
        "hub_address": args.hub_address,
        "serial_port": args.serial_port,
        "baud": args.baud,
        "debug_repl_snippets": args.debug_repl_snippets,
        "motor_port": args.motor_port,
        "scan_timeout": args.scan_timeout,
        "connect_timeout": args.connect_timeout,
    }
    run_server(
        host=args.host,
        port=args.port,
        backend_name=args.backend,
        backend_kwargs=backend_kwargs,
    )


if __name__ == "__main__":
    main()
