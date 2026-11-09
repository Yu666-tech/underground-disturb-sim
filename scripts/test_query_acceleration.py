#!/usr/bin/env python3
"""Compare accelerated mesh queries with the retained exact reference implementation."""
from pathlib import Path
import json
import sys
import time
import unittest
import tempfile
import numpy as np
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/coal_scene_query/scripts'))
from geometry_engine import Engine
from reference_geometry_engine import Engine as ReferenceEngine
from collision_loader import transform


class QueryAcceleration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scene = Path(get_package_share_directory('coal_face_sim')) / 'scenarios/coal_face_v1'
        cls.fast = Engine(scene / 'worlds/coal_face.sdf', scene / 'models')
        cls.reference = ReferenceEngine(scene / 'worlds/coal_face.sdf', scene / 'models')

    def compare(self, origin, target, radius):
        a = self.fast.query(origin, target, radius)
        b = self.reference.query(origin, target, radius)
        self.assertEqual(a[2], b[2])
        for actual, expected, fields in (
                (a[0], b[0], ('distance', 'entry', 'exit', 'normal', 'thickness', 'path_length')),
                (a[1], b[1], ('distance', 'point', 'normal'))):
            self.assertEqual([r['object']['id'] for r in actual], [r['object']['id'] for r in expected])
            for x, y in zip(actual, expected):
                for field in fields:
                    np.testing.assert_allclose(x[field], y[field], atol=1e-8, rtol=1e-10, equal_nan=True)

    def test_controlled_support_clear_and_long_rays(self):
        for radius in (0., .1, 2., 5.):
            for origin, target in (([.6, .75, 1.2], [-2, .75, 1.2]),
                                   ([.6, 5, 1.2105], [.6, 25, 1.2105]),
                                   ([.6, 5, 1.2105], [1.7, 103, 2.1])):
                self.compare(origin, target, radius)

    def test_seeded_rays_across_scene(self):
        rng = np.random.default_rng(20261109)
        for _ in range(60):
            origin = [rng.uniform(-1., 2.), rng.uniform(0., 200.), rng.uniform(.2, 2.5)]
            target = [rng.uniform(-2., 3.), rng.uniform(0., 200.), rng.uniform(.2, 2.5)]
            self.compare(origin, target, float(rng.choice([.01, .5, 2., 5.])))

    def check_box_edges(self, box):
        half = box['size'] / 2
        for offset in (-1e-11, 0., 1e-11):
            local = np.array([[half[0] + offset, -2 * half[1], 0.],
                              [half[0] + offset, 2 * half[1], 0.]])
            world = transform(local, box['matrix'])
            for radius in (0., 2.):
                self.compare(world[0], world[1], radius)
        world = transform(np.array([[0., 0., 0.], [2 * half[0], 0., 0.]]), box['matrix'])
        self.compare(world[0], world[1], 0.)

    def test_box_face_contacts_and_inside_origin(self):
        box = next(o for o in self.fast.objects if o['id'] == 'support_000/link/collision')
        self.check_box_edges(box)

    def test_rotated_box_equivalence(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            world = p / 'world.sdf'
            world.write_text('''<sdf version="1.6"><world name="fixture">
                <model name="shearer"><static>false</static><link name="self"/></model>
                <model name="rotated"><static>true</static><pose>1 2 1 .1 .2 .3</pose><link name="link">
                <collision name="collision"><geometry><box><size>1 2 3</size></box></geometry></collision>
                </link></model></world></sdf>''')
            self.fast = Engine(world, p)
            self.reference = ReferenceEngine(world, p)
            self.check_box_edges(self.fast.objects[0])

    def test_empty_static_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            world = p / 'world.sdf'
            world.write_text('<sdf version="1.6"><world name="empty"><model name="shearer">'
                             '<static>false</static><link name="self"/></model></world></sdf>')
            self.fast = Engine(world, p)
            self.reference = ReferenceEngine(world, p)
            self.compare([0., 0., 0.], [1., 0., 0.], 2.)


if __name__ == '__main__':
    unittest.main()
