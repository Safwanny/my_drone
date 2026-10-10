#!/usr/bin/env python3
"""Realistic GNSS and barometer errors, between gz and everything else.

gz's sensor noise is white only: its Gauss-Markov bias does not evolve for
the navsat and air_pressure sensors, and gz cannot delay a sensor. White
noise averages away, so EKF2 looks better (and behaves differently) than on
real hardware. Here the model's exact navsat_truth / air_pressure_truth
readings get

    GNSS (M10 class): slow error, Gauss-Markov 1.27 m per horizontal axis
                      (1.5 m CEP) and 2 m vertical, tau 120 s, already
                      present at start like a fresh fix; plus white noise;
                      velocity noise; 110 ms delay (= EKF2_GPS_DELAY default)
    barometer:        slow drift, Gauss-Markov 6 Pa (~0.5 m), tau 300 s;
                      plus 3 Pa white noise

and are republished on navsat_sensor / air_pressure_sensor, the topics PX4
and the ROS bridge read. Time is the gz message stamps (sim time).
--seed makes a run repeatable. Not modelled: prop wash and ground effect
on the barometer, multipath steps, outages (campaign fault injection).
"""

import argparse
import collections
import math
import random
import signal
import threading

from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.navsat_pb2 import NavSat
from gz.transport13 import Node

EARTH_RADIUS = 6371000.0  # m


class GaussMarkov:
    """First-order Gauss-Markov process: stationary std sigma, time constant tau."""

    def __init__(self, rng, sigma, tau):
        self.rng, self.sigma, self.tau = rng, sigma, tau
        self.x = rng.gauss(0.0, sigma)  # start in the stationary distribution
        self.t = None

    def at(self, t):
        if self.t is not None and t > self.t:
            a = math.exp(-(t - self.t) / self.tau)
            self.x = a * self.x + self.sigma * math.sqrt(1.0 - a * a) * self.rng.gauss(0.0, 1.0)
        self.t = t
        return self.x


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="empty_flight")
    ap.add_argument("--model", default="build03")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--gnss-delay", type=float, default=0.11, help="s")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    gnss_n = GaussMarkov(rng, 1.27, 120.0)  # m
    gnss_e = GaussMarkov(rng, 1.27, 120.0)
    gnss_u = GaussMarkov(rng, 2.0, 120.0)
    baro = GaussMarkov(rng, 6.0, 300.0)     # Pa
    gauss = rng.gauss

    base = f"/world/{args.world}/model/{args.model}/link/base_link/sensor"
    node = Node()
    navsat_pub = node.advertise(f"{base}/navsat_sensor/navsat", NavSat)
    baro_pub = node.advertise(f"{base}/air_pressure_sensor/air_pressure", FluidPressure)
    lock = threading.Lock()  # gz-transport calls back from its own threads
    queue = collections.deque()  # delayed GNSS messages, oldest first

    def release(now):
        # Called from both sensors, so the 50 Hz barometer sets the delay's
        # resolution (20 ms), not the 10 Hz GNSS.
        while queue and queue[0][0] <= now - args.gnss_delay + 1e-6:
            navsat_pub.publish(queue.popleft()[1])

    def on_navsat(m):
        with lock:
            t = stamp(m)
            lat0 = math.radians(m.latitude_deg)
            north = gnss_n.at(t) + gauss(0.0, 0.15)
            east = gnss_e.at(t) + gauss(0.0, 0.15)
            m.latitude_deg += math.degrees(north / EARTH_RADIUS)
            m.longitude_deg += math.degrees(east / (EARTH_RADIUS * math.cos(lat0)))
            m.altitude += gnss_u.at(t) + gauss(0.0, 0.3)
            m.velocity_north += gauss(0.0, 0.05)
            m.velocity_east += gauss(0.0, 0.05)
            m.velocity_up += gauss(0.0, 0.08)
            queue.append((t, m))
            release(t)

    def on_baro(m):
        with lock:
            t = stamp(m)
            m.pressure += baro.at(t) + gauss(0.0, 3.0)
            baro_pub.publish(m)
            release(t)

    node.subscribe(NavSat, f"{base}/navsat_truth/navsat", on_navsat)
    node.subscribe(FluidPressure, f"{base}/air_pressure_truth/air_pressure", on_baro)

    done = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: done.set())
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    done.wait()


if __name__ == "__main__":
    main()
