"""Gio giat trong mo phong - thuan Python, pytest duoc.

Toc do = trung binh x (1 + giat), giat la qua trinh Ornstein-Uhlenbeck (nhieu co quan tinh: luc
manh luc yeu, doi trong vai giay); huong = huong chinh + mot qua trinh OU cham hon (gio xoay dan).
Ra vector gio (ENU, m/s) de WindEffects cua Gazebo tinh luc tren link (worlds/drone_v2.sdf).
"""

import math
import random


class GustyWind:

    def __init__(self, mean_mps, dir_deg, gust=0.35, gust_tau_s=3.0, dir_sigma_deg=25.0,
                 dir_tau_s=10.0, seed=0):
        """dir_deg: huong gio THOI TOI (ENU, 0 = dong, 90 = bac). gust: do lech chuan cua giat
        theo ti le toc do trung binh."""
        self.mean, self.dir = mean_mps, math.radians(dir_deg)
        self.gust, self.gust_tau = gust, gust_tau_s
        self.dir_sigma, self.dir_tau = math.radians(dir_sigma_deg), dir_tau_s
        self.rng = random.Random(seed)
        self.g = 0.0            # giat (ti le)
        self.d = 0.0            # lech huong (rad)

    @staticmethod
    def _ou(x, tau, sigma, dt, rng):
        """Mot buoc OU chinh xac: giu do lech chuan dung sigma voi moi dt."""
        a = math.exp(-dt / tau)
        return a * x + sigma * math.sqrt(1.0 - a * a) * rng.gauss(0.0, 1.0)

    def step(self, dt):
        """Tien dt giay, tra (vx, vy) gio ENU."""
        self.g = self._ou(self.g, self.gust_tau, self.gust, dt, self.rng)
        self.d = self._ou(self.d, self.dir_tau, self.dir_sigma, dt, self.rng)
        speed = max(0.0, self.mean * (1.0 + self.g))
        a = self.dir + self.d
        return speed * math.cos(a), speed * math.sin(a)
