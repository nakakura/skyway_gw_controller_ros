#!/usr/bin/env python
# -*- coding: utf-8 -*-
PKG = "skyway"
import logging
import rospy
import roslib

roslib.load_manifest(PKG)

import sys
import unittest
import mock
from mock import patch
from os import path
import requests

sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
from scripts.api.data import *


def mocked_requests_get(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_create_data_success/data":
        return MockResponse(args[0], {}, 200)

    return MockResponse(args[0], {}, 410)


def mocked_requests_delete(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_delete_data_success/data/data_id":
        return MockResponse(args[0], {}, 204)

    return MockResponse(args[0], {}, 410)


class TestCreatePeertResp(unittest.TestCase):
    # create peer success
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_create_data_success(self, mock_post):
        response = create_data("url_create_data_success")
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # create peer success
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_create_delete_success(self, mock_delete):
        response = delete_data("url_delete_data_success", DataId("data_id"))
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "peer_api", TestCreatePeertResp)
