"""Regressao da interface: valida criacao e clique em botoes sem pygame instalado.

Este teste usa um substituto minimo de pygame.Rect para verificar o contrato
entre a definicao dos botoes (tuplas) e o desenho/captura de cliques.
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class FakeRect:
    def __init__(self, *args):
        if len(args) == 1:
            data = args[0]
            values = (data.x, data.y, data.w, data.h) if isinstance(data, FakeRect) else data
        else:
            values = args
        self.x, self.y, self.w, self.h = map(int, values)

    @property
    def centerx(self):
        return self.x + self.w // 2

    @property
    def centery(self):
        return self.y + self.h // 2

    def collidepoint(self, point):
        x, y = point
        return self.x <= x < self.x+self.w and self.y <= y < self.y+self.h


class TestButtons(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stub = types.ModuleType('pygame')
        stub.Rect = FakeRect
        stub.mouse = types.SimpleNamespace(get_pos=lambda: (25, 30))
        with patch.dict(sys.modules, {'pygame': stub}):
            spec = importlib.util.spec_from_file_location(
                'robotica_interface_under_test',
                Path(__file__).resolve().parents[1] / 'robotica_interativa.py'
            )
            cls.mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.mod)
        cls.stub = stub

    def setUp(self):
        self.lab = self.mod.RoboLab.__new__(self.mod.RoboLab)
        self.lab.buttons = []
        self.lab.coordinates = lambda pos: pos
        self.lab.help_open = False
        self.lab.world_pos = lambda point: None

    def _draw_button(self, rectangle):
        recorded = []
        called = []
        def fake_text(surf, label, x, y, *args, **kwargs):
            recorded.append((label, x, y))
        with patch.object(self.mod, 'rounded'), patch.object(self.mod, 'text', side_effect=fake_text):
            self.lab.create_button(None, 'TESTE', rectangle, lambda: called.append(True))
        return recorded, called

    def test_accepts_tuple_and_places_label_at_center(self):
        recorded, called = self._draw_button((10, 20, 100, 40))
        self.assertEqual(recorded, [('TESTE', 60, 40)])
        self.assertIsInstance(self.lab.buttons[0][0], FakeRect)
        self.lab.mouse_click((25, 30), 1)
        self.assertEqual(called, [True])

    def test_accepts_rect_and_keeps_hitbox(self):
        recorded, called = self._draw_button(FakeRect(50, 60, 160, 30))
        self.assertEqual(recorded, [('TESTE', 130, 75)])
        self.lab.mouse_click((75, 65), 1)
        self.assertEqual(called, [True])

    def test_does_not_click_outside(self):
        _, called = self._draw_button((10, 20, 100, 40))
        self.lab.mouse_click((500, 500), 1)
        self.assertEqual(called, [])


if __name__ == '__main__':
    unittest.main()
