#!/usr/bin/env python3
"""Deterministic Rao scene geometry used by stage 2 and later planners."""

import json
import math
import random
from dataclasses import dataclass

from rao_2025_protocol import derive_seed


@dataclass(frozen=True)
class Cube:
    center_x: float
    center_y: float
    width: float
    height: float

    @property
    def z_min(self):
        return 0.0

    @property
    def z_max(self):
        return self.height

    def as_dict(self):
        return {
            "type": "cube",
            "center": [self.center_x, self.center_y, self.height / 2.0],
            "size": [self.width, self.width, self.height],
        }


@dataclass(frozen=True)
class Annulus:
    center_x: float
    center_y: float
    center_z: float
    outer_radius: float
    inner_radius: float
    tilt_rad: float
    wall_thickness: float

    def as_dict(self):
        return {
            "type": "annulus",
            "center": [self.center_x, self.center_y, self.center_z],
            "outer_radius": self.outer_radius,
            "inner_radius": self.inner_radius,
            "tilt_rad": self.tilt_rad,
            "wall_thickness": self.wall_thickness,
        }


class Scene:
    def __init__(self, profile, paper_profile, reproduction, master_seed, protocol_sha256):
        self.profile = profile
        self.protocol_sha256 = protocol_sha256
        self.size_x, self.size_y, self.size_z = paper_profile["scene_size_m"]
        self.obstacle_count = paper_profile["obstacle_count"]
        common = reproduction["_paper_common"]
        self.cube_width_min, self.cube_width_max = common["cube_width_range_m"]
        self.cube_height_min, self.cube_height_max = common["cube_height_range_m"]
        self.annulus_diameter_min, self.annulus_diameter_max = common["annulus_diameter_range_m"]
        self.tilt_choices = common["annulus_tilt_rad"]
        self.annulus_fraction = reproduction["scene_generation"]["annulus_fraction"]
        self.wall_thickness = reproduction["scene_generation"]["annulus_wall_thickness_m"]
        self.annulus_center_margin = reproduction["scene_generation"]["annulus_center_margin_m"]
        self.rng = random.Random(derive_seed(master_seed, profile, "scene"))
        self.obstacles = self._generate()

    def _inside_bounds(self, x, y, margin):
        return (-self.size_x / 2.0 + margin <= x <= self.size_x / 2.0 - margin and
                -self.size_y / 2.0 + margin <= y <= self.size_y / 2.0 - margin)

    def _generate(self):
        annulus_count = int(round(self.obstacle_count * self.annulus_fraction))
        obstacles = []
        attempts = 0
        while len(obstacles) < self.obstacle_count and attempts < self.obstacle_count * 100:
            attempts += 1
            is_annulus = len(obstacles) < annulus_count
            if is_annulus:
                diameter = self.rng.uniform(self.annulus_diameter_min, self.annulus_diameter_max)
                outer = diameter / 2.0
                margin = self.annulus_center_margin
                x = self.rng.uniform(-self.size_x / 2.0 + outer + margin,
                                     self.size_x / 2.0 - outer - margin)
                y = self.rng.uniform(-self.size_y / 2.0 + outer + margin,
                                     self.size_y / 2.0 - outer - margin)
                z = self.rng.uniform(outer + margin, self.size_z - outer - margin)
                inner = max(0.0, outer - self.wall_thickness)
                tilt = self.rng.choice(self.tilt_choices)
                obstacles.append(Annulus(x, y, z, outer, inner, tilt, self.wall_thickness))
            else:
                width = self.rng.uniform(self.cube_width_min, self.cube_width_max)
                height = self.rng.uniform(self.cube_height_min, min(self.cube_height_max, self.size_z))
                x = self.rng.uniform(-self.size_x / 2.0 + width / 2.0, self.size_x / 2.0 - width / 2.0)
                y = self.rng.uniform(-self.size_y / 2.0 + width / 2.0, self.size_y / 2.0 - width / 2.0)
                obstacles.append(Cube(x, y, width, height))
        if len(obstacles) != self.obstacle_count:
            raise RuntimeError("could not generate the requested deterministic obstacle count")
        return obstacles

    def as_dict(self):
        return {
            "profile": self.profile,
            "protocol_sha256": self.protocol_sha256,
            "scene_size_m": [self.size_x, self.size_y, self.size_z],
            "obstacle_count": len(self.obstacles),
            "obstacles": [obstacle.as_dict() for obstacle in self.obstacles],
        }

    def as_json(self):
        return json.dumps(self.as_dict(), sort_keys=True, separators=(",", ":"))

    def point_collision(self, x, y, z, clearance=0.0):
        for obstacle in self.obstacles:
            if isinstance(obstacle, Cube):
                if (abs(x - obstacle.center_x) <= obstacle.width / 2.0 + clearance and
                        abs(y - obstacle.center_y) <= obstacle.width / 2.0 + clearance and
                        obstacle.z_min - clearance <= z <= obstacle.z_max + clearance):
                    return True
            else:
                # The annulus is a vertical ring. Its axis lies in the xy plane;
                # tilt rotates that axis about +z from the +x direction.
                dx, dy, dz = x - obstacle.center_x, y - obstacle.center_y, z - obstacle.center_z
                axis_x, axis_y = math.cos(obstacle.tilt_rad), math.sin(obstacle.tilt_rad)
                along = dx * axis_x + dy * axis_y
                lateral = -dx * axis_y + dy * axis_x
                radial = math.hypot(lateral, dz)
                if abs(along) <= obstacle.wall_thickness / 2.0 + clearance and \
                        obstacle.inner_radius - clearance <= radial <= obstacle.outer_radius + clearance:
                    return True
        return False


def make_scene(protocol, profile):
    paper = protocol["paper"]
    reproduction = dict(protocol["reproduction"])
    reproduction["_paper_common"] = paper["common_parameters"]
    return Scene(profile, paper["profiles"][profile], reproduction,
                 protocol["reproduction"]["master_seed"], "direct_protocol")


def make_scene_from_manifest(manifest):
    reproduction = {
        "_paper_common": manifest["paper_common_parameters"],
        "scene_generation": manifest["scene_generation"],
    }
    return Scene(manifest["profile"], manifest["paper_scene"], reproduction,
                 manifest["master_seed"], manifest["config_sha256"])


def scene_from_dict(data):
    """Reconstruct the immutable geometry received from rao_scene_node."""
    scene = object.__new__(Scene)
    scene.profile = data["profile"]
    scene.protocol_sha256 = data["protocol_sha256"]
    scene.size_x, scene.size_y, scene.size_z = data["scene_size_m"]
    scene.obstacle_count = data["obstacle_count"]
    obstacles = []
    for item in data["obstacles"]:
        if item["type"] == "cube":
            obstacles.append(Cube(item["center"][0], item["center"][1], item["size"][0], item["size"][2]))
        elif item["type"] == "annulus":
            obstacles.append(Annulus(item["center"][0], item["center"][1], item["center"][2],
                                     item["outer_radius"], item["inner_radius"], item["tilt_rad"],
                                     item["wall_thickness"]))
        else:
            raise ValueError("unknown obstacle type: {}".format(item["type"]))
    if len(obstacles) != scene.obstacle_count:
        raise ValueError("scene obstacle count does not match geometry payload")
    scene.obstacles = obstacles
    return scene
