"""Restart, gaps, discarded reservations, and actual camera-name regression tests."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

from gui.episode_catalog import progress, reserve
from gui.legacy_backend import LegacyBackend
from gui.session import ControlLoop


class CatalogTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def kept(self, name):
        p = self.root / name
        p.mkdir()
        (p / 'episode.csv').write_text('t_ns,q\n1,0\n')
        (p / 'episode_meta.json').write_text('{"frames": 1}')
        return p

    def test_restart_counts_kept_and_does_not_fill_id_gaps(self):
        self.kept('ep001'); self.kept('ep008')
        self.kept('other012')
        (self.root / 'ep004').mkdir()
        b = LegacyBackend(episodes_dir=self.root)
        state = ControlLoop(b).snapshot()
        self.assertEqual(state.episode_index, 2)
        self.assertEqual(state.incomplete_episodes, 1)
        self.assertEqual(state.next_episode, 'ep009')
        self.assertEqual(reserve(self.root, 'ep').name, 'ep009')

    def test_discarded_id_is_not_reused_after_restart(self):
        p = reserve(self.root, 'ep')
        self.assertEqual(p.name, 'ep001')
        shutil.rmtree(p)
        state = ControlLoop(LegacyBackend(episodes_dir=self.root)).snapshot()
        self.assertEqual(state.episode_index, 0)
        self.assertEqual(state.next_episode, 'ep002')
        self.assertEqual(reserve(self.root, 'ep').name, 'ep002')

    def test_incomplete_failed_and_symlink_episodes_are_not_counted(self):
        ep = self.kept('ep001')
        (ep / 'failure.json').write_text('{}')
        ep2 = self.kept('ep002'); (ep2 / 'episode.csv').write_text('header\n')
        self.kept('ep003'); (self.root / 'ep003' / 'episode_meta.json').write_text('{bad')
        (self.root / 'ep999').symlink_to(ep, target_is_directory=True)
        stats = progress(self.root, 'ep')
        self.assertEqual(stats['episode_index'], 0)
        self.assertEqual(stats['next_episode'], 'ep1000')

    def test_prefixes_have_independent_sequences_and_existing_files_are_reserved(self):
        self.kept('ep008')
        (self.root / 'ep009').write_text('occupied')
        self.assertEqual(reserve(self.root, 'ep').name, 'ep010')
        self.assertEqual(reserve(self.root, 'demo').name, 'demo001')
        self.assertEqual((self.root / 'ep009').read_text(), 'occupied')

    def test_bad_sequence_or_prefix_refuses_to_reuse_ids(self):
        with self.assertRaises(ValueError):
            reserve(self.root, '../ep')
        (self.root / '.pnp7_episode_sequence.json').write_text('{"ep": "bad"}')
        with self.assertRaises(ValueError):
            reserve(self.root, 'ep')

    def test_real_camera_name_is_passed_to_builder(self):
        ep = self.kept('ep001')
        for role in ('cam2022', 'wrist'):
            (ep / f'cam_{role}_index.csv').write_text('index')
        b = LegacyBackend(episodes_dir=self.root)
        with patch('gui.legacy_backend.subprocess.run', return_value=SimpleNamespace(
                returncode=0, stdout='ok', stderr='')) as run, patch.object(b, '_validate', return_value={'verdict':'PASS'}):
            report = b._build_and_validate(ep)
        args = run.call_args.args[0]
        self.assertEqual(args[args.index('--anchor') + 1], 'cam2022')
        self.assertEqual(report['frames'], 1)

    def test_failed_build_does_not_count_as_kept(self):
        ep = self.root / 'ep001'; ep.mkdir()
        b = LegacyBackend(episodes_dir=self.root)
        b._episode_dir = ep
        with patch.object(b, '_teleop_rows', return_value=2000), patch.object(b, '_build_and_validate', return_value={'verdict':'FAIL','build_output':['anchor missing']}):
            with self.assertRaisesRegex(RuntimeError, '汇总生成失败'):
                b.end_episode(keep=True)
        self.assertEqual(b._episode_dir, ep)
        self.assertTrue(ep.exists())
        self.assertEqual(progress(self.root,'ep')['episode_index'], 0)

if __name__ == '__main__':
    unittest.main()
