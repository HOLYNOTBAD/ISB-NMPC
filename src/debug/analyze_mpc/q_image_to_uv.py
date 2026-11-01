#!/usr/bin/env python3

import argparse
import math


def normalize_quaternion(qw, qx, qy, qz):
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("q_image norm is zero or invalid")
    return qw / norm, qx / norm, qy / norm, qz / norm


def q_image_to_ray(qw, qx, qy, qz):
    qw, qx, qy, qz = normalize_quaternion(qw, qx, qy, qz)
    return (
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def ray_to_uv(ray, img_f, cx, cy):
    x, y, z = ray
    if z <= 1e-6:
        raise ValueError(
            "LOS ray z={:.9f} is not projectable in front of the camera".format(z)
        )
    return cx + img_f * x / z, cy + img_f * y / z


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert q_image quaternion to image pixel coordinates u, v."
    )
    parser.add_argument("qw", type=float, help="q_image quaternion w")
    parser.add_argument("qx", type=float, help="q_image quaternion x")
    parser.add_argument("qy", type=float, help="q_image quaternion y")
    parser.add_argument("qz", type=float, help="q_image quaternion z")
    parser.add_argument("--img-width", type=float, default=640.0)
    parser.add_argument("--img-height", type=float, default=480.0)
    parser.add_argument("--img-f", type=float, default=115.0)
    parser.add_argument("--cx", type=float, default=None)
    parser.add_argument("--cy", type=float, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    cx = 0.5 * args.img_width if args.cx is None else args.cx
    cy = 0.5 * args.img_height if args.cy is None else args.cy

    ray = q_image_to_ray(args.qw, args.qx, args.qy, args.qz)
    u, v = ray_to_uv(ray, args.img_f, cx, cy)

    print("ray_camera: x={:+.9f}, y={:+.9f}, z={:+.9f}".format(*ray))
    print("u={:.3f}".format(u))
    print("v={:.3f}".format(v))


if __name__ == "__main__":
    main()
