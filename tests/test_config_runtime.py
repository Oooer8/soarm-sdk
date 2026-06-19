from __future__ import annotations

import tempfile
import unittest
from contextlib import contextmanager
from os import chdir, getcwd
from pathlib import Path

from soarm_sdk import SOARMConfig, default_config_path, ensure_user_config, resolve_config_path


@contextmanager
def _cwd(path: Path):
    previous = getcwd()
    chdir(path)
    try:
        yield
    finally:
        chdir(previous)


class ConfigRuntimeTest(unittest.TestCase):
    def test_packaged_default_loads(self) -> None:
        config = SOARMConfig.from_file(default_config_path())

        self.assertEqual(config.arm.control_hz, 200)
        self.assertEqual(config.arm.low_voltage, 7.0)
        self.assertIn("shoulder_pan", config.joints)

    def test_ensure_user_config_copies_split_files(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "robot" / "soarm-sdk.yaml"

            result = ensure_user_config(output)

            self.assertEqual(result, output)
            self.assertTrue(output.is_file())
            self.assertTrue((output.parent / "runtime.yaml").is_file())
            self.assertTrue((output.parent / "motors" / "feetech_sts3215.yaml").is_file())
            config = SOARMConfig.from_file(output)
            self.assertEqual(config.source.runtime_path, output.parent / "runtime.yaml")
            self.assertEqual(config.arm.low_voltage, 7.0)

    def test_resolve_config_prefers_existing_user_copy(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with _cwd(root):
                self.assertEqual(resolve_config_path(), default_config_path())
                output = ensure_user_config()
                self.assertEqual(resolve_config_path(), output)

    def test_resolve_config_generates_missing_explicit_write_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "custom" / "arm.yaml"

            self.assertEqual(resolve_config_path(output, for_write=True), output)

            self.assertTrue(output.is_file())
            self.assertTrue((output.parent / "runtime.yaml").is_file())


if __name__ == "__main__":
    unittest.main()
