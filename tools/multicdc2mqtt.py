#!/usr/bin/env python3
"""A tool that enables control of multicdc LEDs via MQTT."""
import argparse
import paho.mqtt.client as mqtt
import serial
import logging
import json
import re

_LOGGER = logging.getLogger(__name__)

WW_KELVIN = 3044
CW_KELVIN = 6675


def main():
    parser = argparse.ArgumentParser(
            add_help=False,  # avoids conflict with -h for hostname
            description='MQTT bridge for multicdc LEDs')

    parser.add_argument(
        '--help', '-H', action='help',
        help='show this help message and exit'
    )

    parser.add_argument(
        '--hostname', '-h', default='localhost',
        help="MQTT broker host (default: %(default)s)"
    )

    parser.add_argument(
        '--port', '-p', type=int, default=1883,
        help="MQTT broker port (default: %(default)s)"
    )

    parser.add_argument(
        '--topic', '-t', default='multicdc',
        help="MQTT topic prefix (default: %(default)s)"
    )

    parser.add_argument(
        '--username', '-u', default=None,
        help='MQTT username (default: anonymous login)'
    )

    parser.add_argument(
        '--password', '-P', default="",
        help="MQTT password (default: '%(default)s')"
    )

    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='log debug level messages'
    )

    parser.add_argument(
        "--device-id", type=str, default=None,
        help="device id for Home Assistant discovery (default: no discovery message sent)"
    )

    parser.add_argument(
        'device', type=str,
        help='serial port to use (e.g. /dev/tty-mcdc0shell)'
    )

    parser.add_argument(
        '--baudrate', '-b', type=int, default=9600,
        help="baudrate to be used with the serial port (default: %(default)s)"
    )

    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    ser = serial.Serial(args.device, baudrate=args.baudrate)

    def ser_tx(cmd: str) -> None:
        _LOGGER.debug("tx: %r", cmd)
        ser.write(cmd.encode('ascii'))

    def rgbcct(r: int, g: int, b: int, c: int, w: int, tran: int) -> None:
        ser_tx(f"\nrgbcct {r} {g} {b} {c} {w} {tran}\n")

    def backlight(brightness: int, tran: int) -> None:
        ser_tx(f"\nbacklight {brightness} {tran}\n")

    def on_message_rgbcct(mqttc, obj, msg) -> None:
        try:
            _LOGGER.debug("mqtt %s: %r", msg.topic, msg.payload)
            j = json.loads(msg.payload.decode('ascii'))
            tran_default = 0.2
            if j['state'] == "ON":
                col = j.get('color', {})
                r = int(col.get('r', 0))
                g = int(col.get('g', 0))
                b = int(col.get('b', 0))
                c = int(col.get('c', 0))
                w = int(col.get('w', j.get('brightness', 200)))
                if not col and "brightness" not in j:
                    # only ON command was sent, let's do it gradually
                    tran_default = 2
            else:
                r = 0
                g = 0
                b = 0
                c = 0
                w = 0
                tran_default = 2
            tran = int(j.get('transition', tran_default) * 1000)
            rgbcct(r, g, b, c, w, tran)
        except Exception:
            _LOGGER.exception("exception in on_message_rgbcct")

    def on_message_backlight(mqttc, obj, msg) -> None:
        try:
            _LOGGER.debug("mqtt %s: %r", msg.topic, msg.payload)
            j = json.loads(msg.payload.decode('ascii'))
            tran_default = 0.2
            if j["state"] == "ON":
                if "brightness" in j:
                    brightness = int(j["brightness"])
                else:
                    brightness = 255
                    tran_default = 2
            else:
                brightness = 0
                tran_default = 2
            tran = int(j.get('transition', tran_default) * 1000)
            backlight(brightness, tran)
        except Exception:
            _LOGGER.exception("exception in on_message_backlight")

    TOPICS = {
        f"{args.topic}/cmnd/rgbcct": on_message_rgbcct,
        f"{args.topic}/cmnd/backlight": on_message_backlight,
    }

    def on_connect(mqttc, obj, flags, reason_code, properties):
        _LOGGER.info("on_connect rc=%r", reason_code)
        if reason_code == 135:
            raise Exception("Incorrect MQTT login credentials")
        for topic in TOPICS:
            mqttc.subscribe(topic)
        ser_tx("\nledstate\n")  # read LED states

    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect
    for topic, handler in TOPICS.items():
        mqttc.message_callback_add(topic, handler)
    mqttc.will_set(f"{args.topic}/availability", "offline", retain=True)
    if args.username:
        mqttc.username_pw_set(args.username, args.password)

    with ser:
        # only start receiving mqtt commands after the serial port is open
        mqttc.connect(args.hostname, args.port)
        # send MQTT discovery message
        if args.device_id:
            _LOGGER.info("sending discovery message")
            mqttc.publish(
                f"homeassistant/device/{args.device_id}/config",
                json.dumps({
                    "dev": {
                        "ids": args.device_id,
                        "name": "multicdc LEDs",
                    },
                    "o": {
                        "name": "multicdc2mqtt",
                    },
                    "cmps": {
                        "rgbcct": {
                            "p": "light",
                            "name": "RGBCCT",
                            "unique_id": f"{args.device_id}_rgbcct",
                            "supported_color_modes": ["rgbww"],
                            "brightness": False,
                            "schema": "json",
                            "min_k": WW_KELVIN,
                            "max_k": CW_KELVIN,
                            "command_topic": f"{args.topic}/cmnd/rgbcct",
                            "state_topic": f"{args.topic}/stat/rgbcct",
                        },
                        "backlight": {
                            "p": "light",
                            "name": "backlight",
                            "unique_id": f"{args.device_id}_backlight",
                            "supported_color_modes": ["brightness"],
                            "schema": "json",
                            "command_topic": f"{args.topic}/cmnd/backlight",
                            "state_topic": f"{args.topic}/stat/backlight"
                        },
                    },
                    "avty_t": f"{args.topic}/availability",
                    }),
                retain=True
            )

        try:
            mqttc.loop_start()

            while True:
                line = ser.readline().decode('ascii').strip()
                if not line:
                    continue
                _LOGGER.debug("rx line: %r", line)
                if m := re.fullmatch(
                        r"!rgbcct r=(?P<r>\d+) g=(?P<g>\d+) b=(?P<b>\d+)"
                        r" c=(?P<c>\d+) w=(?P<w>\d+)"
                        r" duration_ms=(?P<duration_ms>\d+)",
                        line):
                    r = int(m.group("r"))
                    g = int(m.group("g"))
                    b = int(m.group("b"))
                    c = int(m.group("c"))
                    w = int(m.group("w"))
                    msg = json.dumps({
                        "state": "ON" if max(r, g, b, c, w) > 0 else "OFF",
                        "color_mode": "rgbww",
                        "color": {
                            "r": r,
                            "g": g,
                            "b": b,
                            "c": c,
                            "w": w,
                        },
                        "brightness": max(r, g, b, c, w),
                    })
                    mqttc.publish(f"{args.topic}/stat/rgbcct", msg.encode('ascii'), retain=True)
                elif m := re.fullmatch(
                        r"!backlight duty=(?P<duty>\d+) duration_ms=(?P<duration_ms>\d+)",
                        line):
                    brightness = int(m.group("duty"))
                    msg = json.dumps({
                        "state": "ON" if brightness > 0 else "OFF",
                        "brightness": brightness,
                    })
                    mqttc.publish(f"{args.topic}/stat/backlight", msg.encode('ascii'), retain=True)

                elif line == "!ledstate":
                    # full LED state info received, we only do this after MQTT
                    # reconnect
                    _LOGGER.debug("publishing availability online")
                    mqttc.publish(f"{args.topic}/availability", "online", retain=True)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
