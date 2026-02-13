#!/usr/bin/env python3
import math
import argparse

def gamma_correction(i: float, gamma: float, max_in: int, max_out: int) -> int:
    return int(pow(i / max_in, gamma) * max_out + 0.5)

def generate_lut(name: str, max_in: int, max_out: int):
    lut = [
        gamma_correction(i, 2.8, max_in, max_out) for i in range(max_in+1)
    ]
    lut_str = (f"static const {'uint8_t' if max_out < 256 else 'uint16_t'} {name}[] = {{\n"
               f"    {',\n    '.join(map(str, lut))}\n}};")
    return lut_str


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("max_in", type=int)
    parser.add_argument("max_out", type=int)
    args = parser.parse_args()

    i = args.max_in
    o = args.max_out
    print(generate_lut(f"gamma{i}_{o}", i, o));
