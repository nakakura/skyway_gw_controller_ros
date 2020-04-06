#!/usr/bin/env python
# -*- coding: utf-8 -*-
PKG = "skyway"
import roslib

roslib.load_manifest(PKG)

import sys
import unittest
from os import path

sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
from scripts.api.peer import *


class TestPeer(unittest.TestCase):
    def test_one_equals_one(self):  # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "skyway", TestPeer)
