"""Exercise Git selection and the actual pinned formatter without editing the repo."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("format_cpp", ROOT / "bin/format_cpp.py")
fmt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fmt)


class FormatTests(unittest.TestCase):
    def test_exclusions_and_extensions(self):
        for name in ('third_party/x.h', './third_party/x.h', 'thirdparty/flatbuffers/a.h', 'plugin/usdNoodles/bindings.cpp',
                     'libs/rigExecBinary/generated/a.h', 'build/a.cpp', 'docs/x.md'):
            self.assertFalse(fmt.eligible(name), name)
        self.assertTrue(fmt.eligible('libs/rigExec/a.cpp'))

    def test_selection_includes_committed_staged_unstaged_new_and_renamed(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            def git(*args):
                return subprocess.check_output(['git', *args], cwd=root, stderr=subprocess.DEVNULL)
            git('init', '-b', 'main')
            git('config', 'user.name', 'Test')
            git('config', 'user.email', 'test@example.invalid')
            for name in ('committed.cpp', 'staged.cpp', 'working.cpp', 'old.cpp', 'deleted.cpp', 'unchanged.cpp'):
                (root / name).write_text('int x;\n')
            git('add', '.')
            git('commit', '-m', 'base')
            base = git('rev-parse', 'HEAD').decode().strip()
            (root / 'committed.cpp').write_text('int committed;\n')
            git('add', 'committed.cpp')
            git('commit', '-m', 'candidate')
            (root / 'staged.cpp').write_text('int staged;\n')
            git('add', 'staged.cpp')
            (root / 'working.cpp').write_text('int working;\n')
            (root / 'new file.cpp').write_text('int fresh;\n')
            git('mv', 'old.cpp', 'renamed.cpp')
            git('rm', 'deleted.cpp')
            self.assertEqual(fmt.select_files(root, base),
                ['committed.cpp', 'new file.cpp', 'renamed.cpp', 'staged.cpp', 'working.cpp'])
            with self.assertRaises(ValueError):
                fmt.select_files(root, paths=['../outside.cpp'])

    def test_wrong_version_is_rejected(self):
        with patch.object(fmt.shutil, 'which', return_value='clang-format'), \
             patch.object(fmt.subprocess, 'check_output', return_value='clang-format version 19.1.0'):
            with self.assertRaisesRegex(ValueError, '18.1.8'):
                fmt.formatter()

    @unittest.skipUnless(shutil.which('clang-format'), 'install clang-format 18.1.8 for integration test')
    def test_check_fix_and_idempotence_with_usd_macros(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            shutil.copyfile(ROOT / '.clang-format', root / '.clang-format')
            source = root / 'test.cpp'
            source.write_text('#include "z.h"\n#include "a.h"\n\nPXR_NAMESPACE_OPEN_SCOPE\n'
                              'int Test(int x){if(x){return x+1;}return 0;}\nPXR_NAMESPACE_CLOSE_SCOPE\n')
            original = source.read_bytes()
            self.assertEqual(fmt.main(['--repo', d, 'test.cpp']), 1)
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(fmt.main(['--repo', d, '--fix', 'test.cpp']), 0)
            output = source.read_text()
            self.assertIn('int\nTest(int x)\n{\n    if (x) {\n', output)
            self.assertLess(output.index('"z.h"'), output.index('"a.h"'))
            self.assertIn('\nPXR_NAMESPACE_CLOSE_SCOPE\n', output)
            self.assertEqual(fmt.main(['--repo', d, 'test.cpp']), 0)


if __name__ == '__main__':
    unittest.main()
