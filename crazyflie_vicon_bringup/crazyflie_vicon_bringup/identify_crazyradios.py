"""Identify eight Crazyradio USB dongles by exercising their activity LEDs."""

import argparse
import time


USB_VENDOR_ID = 0x1915
USB_PRODUCT_ID = 0x7777
RADIO_COUNT = 8
OUT_ENDPOINT = 0x01
IN_ENDPOINT = 0x81
SET_RADIO_CHANNEL = 0x01
SET_RADIO_ADDRESS = 0x02
SET_RADIO_DATA_RATE = 0x03
ISOLATED_CHANNEL = 125
DATA_RATE_2M = 2
TRAFFIC_PERIOD = 0.02


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Blink the activity LED on Crazyradios 1 through 8 in USB '
            'enumeration order. Do not run crazyflie_server at the same time.'
        )
    )
    parser.add_argument(
        '--seconds',
        type=float,
        default=3.0,
        help='seconds to identify each radio (default: 3.0)',
    )
    return parser.parse_args()


def radio_label(device):
    return f'bus {device.bus}, device {device.address}'


def configure_isolated_link(device, index):
    address = bytes([0xA0 + index, 0x5A, 0xC3, 0x7E, 0x19])
    device.set_configuration()
    device.ctrl_transfer(0x40, SET_RADIO_CHANNEL, ISOLATED_CHANNEL, 0, None)
    device.ctrl_transfer(0x40, SET_RADIO_ADDRESS, 0, 0, address)
    device.ctrl_transfer(0x40, SET_RADIO_DATA_RATE, DATA_RATE_2M, 0, None)


def exercise_activity_led(device, duration, usb_core):
    deadline = time.monotonic() + duration
    packet = bytes([0xFF])
    while time.monotonic() < deadline:
        device.write(OUT_ENDPOINT, packet, timeout=100)
        try:
            device.read(IN_ENDPOINT, 64, timeout=20)
        except usb_core.USBTimeoutError:
            pass
        time.sleep(TRAFFIC_PERIOD)


def main():
    args = parse_args()
    if args.seconds <= 0.0:
        raise SystemExit('--seconds must be greater than zero.')

    try:
        import usb.core
        import usb.util
    except ImportError as exc:
        raise SystemExit(
            'PyUSB is required. Install it with: sudo apt install python3-usb'
        ) from exc

    radios = sorted(
        usb.core.find(
            find_all=True,
            idVendor=USB_VENDOR_ID,
            idProduct=USB_PRODUCT_ID,
        ),
        key=lambda device: (device.bus, device.address),
    )
    if len(radios) != RADIO_COUNT:
        raise SystemExit(
            f'Expected {RADIO_COUNT} Crazyradios, but found {len(radios)}. '
            'Check USB connections and stop crazyflie_server.'
        )

    print('Crazyradio numbering follows the USB bus/device order shown below.')
    try:
        for index, radio in enumerate(radios, start=1):
            print(
                f'Crazyradio {index}: {radio_label(radio)}; '
                f'activity LED for {args.seconds:g} seconds...',
                flush=True,
            )
            try:
                configure_isolated_link(radio, index)
                exercise_activity_led(radio, args.seconds, usb.core)
            except usb.core.USBError as exc:
                raise SystemExit(
                    f'Could not access Crazyradio {index} ({radio_label(radio)}): '
                    f'{exc}. Stop crazyflie_server and check USB permissions.'
                ) from exc
            finally:
                usb.util.dispose_resources(radio)
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
        return

    print('Finished identifying Crazyradios 1 through 8.', flush=True)


if __name__ == '__main__':
    main()
