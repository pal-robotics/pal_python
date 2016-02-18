#!/usr/bin/env python

import subprocess
import tempfile
import os
import signal

class ShellCmd:

    def __init__(self, cmd):
        self.outf = tempfile.NamedTemporaryFile(mode="w")
        self.errf = tempfile.NamedTemporaryFile(mode="w")
        self.inf = tempfile.NamedTemporaryFile(mode="r")
        self.process = subprocess.Popen(cmd, shell=True, stdin=self.inf,
                                        stdout=self.outf, stderr=self.errf,
                                         preexec_fn=os.setsid)

    def __del__(self):
        if not self.is_done():
            self.kill()
        self.outf.close()
        self.errf.close()
        self.inf.close()

    def get_stdout(self):
        with open(self.outf.name, "r") as f:
            return f.read()

    def get_stderr(self):
        with open(self.errf.name, "r") as f:
            return f.read()

    def get_retcode(self):
        """get retcode or None if still running"""
        return self.process.poll()

    def is_done(self):
        return self.process.poll() != None

    def kill(self):
        os.killpg(self.process.pid, signal.SIGTERM)
        self.process.wait()
