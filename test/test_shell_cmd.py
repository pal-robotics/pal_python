#!/usr/bin/env python

import unittest
import time
from pal_python import shell_cmd


class TestShellCmd(unittest.TestCase):

    def __init__(self, *args):
        #unittest.TestCase(*args)
        super(TestShellCmd, self).__init__(*args)

    def test_short_cmd(self):
        cmd = "echo $((2 + 3))"
        shcmd = shell_cmd.ShellCmd(cmd)
        while not shcmd.is_done():
            pass
        self.assertTrue(shcmd.is_done())
        self.assertEqual(shcmd.get_retcode(), 0)
        self.assertEqual(shcmd.get_stdout(), "5\n")
        #Assert out can be retrieved multiple times
        self.assertEqual(shcmd.get_stdout(), "5\n")
        self.assertEqual(shcmd.get_stderr(), "")

    def test_abortable_cmd(self):
        cmd = "yes"
        shcmd = shell_cmd.ShellCmd(cmd)
        self.assertFalse(shcmd.is_done())
        self.assertIsNone(shcmd.get_retcode())
        for i in range(0,4):
            time.sleep(0.2)
            self.assertFalse(shcmd.is_done())
        shcmd.kill()
        self.assertTrue(shcmd.is_done())
        self.assertIsNotNone(shcmd.get_retcode())
        self.assertNotEqual(shcmd.get_retcode(), 0)
        self.assertEqual(shcmd.get_stderr(), "")

    def test_erro_cmd(self):
        cmd = "ls /usr && ls non_existing_file"
        shcmd = shell_cmd.ShellCmd(cmd)
        while not shcmd.is_done():
            pass
        self.assertTrue(shcmd.is_done())
        self.assertIsNotNone(shcmd.get_retcode())
        self.assertNotEqual(shcmd.get_retcode(), 0)
        self.assertNotEqual(shcmd.get_stdout(), "")
        self.assertNotEqual(shcmd.get_stderr(), "")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pal_python', 'test_shell_cmd', TestShellCmd)

