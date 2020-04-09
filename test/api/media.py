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
from scripts.api.media import *


def mocked_requests_post(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_create_media/media":
        return MockResponse(args[0], {}, 201)
    elif args[0] == "url_create_rtcp/media/rtcp":
        return MockResponse(args[0], {}, 201)

    return MockResponse(args[0], {}, 410)


def mocked_requests_delete(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_delete_media/media/media_id":
        return MockResponse(args[0], {}, 204)

    return MockResponse(args[0], {}, 410)


class TestMediaApi(unittest.TestCase):
    # open MediaSocket
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_create_media_success(self, mock_post):
        response = create_media("url_create_media", True)
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # delete MediaSocket
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_create_media_success(self, mock_post):
        response = delete_media("url_delete_media", MediaId("media_id"))
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # create RtcpSocket
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_create_rtcp_success(self, mock_post):
        response = create_rtcp("url_create_rtcp")
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "media_api", TestMediaApi)
