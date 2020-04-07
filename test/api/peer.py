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
from scripts.api.peer import *


def mocked_requests_post(*args, **kwargs):
    class MockResponse:
        def __init__(self, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code

        def json(self):
            return self.json_data

    if args[0] == "url_create_peer_success/peers":
        return MockResponse(kwargs["json"], 201)
    elif args[0] == "url_create_peer_fail/peers":
        return MockResponse({}, 404)

    return MockResponse({}, 410)


class TestCreatePeertResp(unittest.TestCase):
    # create peer success
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_create_peer_success(self, mock_post):
        response = create_peer(
            "url_create_peer_success", "key", "domain", PeerId("peerid"), True
        )
        self.assertEqual(
            response.json(),
            {"turn": True, "domain": "domain", "peer_id": "peerid", "key": "key"},
        )
        self.assertEqual(response.is_ok(), True)

    # create peer gets invalid response
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_create_peer_fail(self, mock_post):
        response = create_peer(
            "url_create_peer_fail", "key", "domain", PeerId("peerid"), True
        )
        self.assertEqual(
            response.err(),
            {"url": "url_create_peer_fail/peers", "error": "404 Not Found"},
        )
        self.assertEqual(response.is_ok(), False)

    # create peer fail due to http error
    def test_create_peer_no_server(self):
        response = create_peer(
            "http://url_create_peer_fail", "key", "domain", PeerId("peerid"), True
        )
        self.assertEqual(
            isinstance(response.err(), requests.exceptions.RequestException), True
        )
        self.assertEqual(response.is_ok(), False)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "peer_api", TestCreatePeertResp)
