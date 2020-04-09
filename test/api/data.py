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
    elif (
        args[0]
        == "url_delete_data_connection_success/data/connections/data_connection_id"
    ):
        return MockResponse(args[0], {}, 204)

    return MockResponse(args[0], {}, 410)


def mocked_requests_post(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_connections_success/data/connections":
        return MockResponse(args[0], kwargs["json"], 202)

    return MockResponse(args[0], {}, 410)


def mocked_requests_put(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_redirect_success/data/connections/data_connection_id":
        return MockResponse(args[0], kwargs["json"], 200)

    return MockResponse(args[0], {}, 410)


class TestCreatePeertResp(unittest.TestCase):
    # open dataport
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_create_data_success(self, mock_post):
        response = create_data("url_create_data_success")
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # close dataport
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_create_delete_success(self, mock_delete):
        response = delete_data("url_delete_data_success", DataId("data_id"))
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # establish dataconnection success
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_create_delete_success(self, mock_post):
        options = DataConnectionOptions(
            PeerInfo("peer_id", "token"),
            {
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "high",
                },
            },
            "target_id",
            DataId("data_id"),
            {"ip_v4": "127.0.0.1", "port": 10001},
        )
        response = connect("url_connections_success", options)
        val = {
            "peer_id": "peer_id",
            "token": "token",
            "options": {
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "high",
                },
            },
            "target_id": "target_id",
            "params": {"data_id": "data_id"},
            "redirect_params": {"ip_v4": "127.0.0.1", "port": 10001,},
        }
        self.assertEqual(response.json(), val)
        self.assertEqual(response.is_ok(), True)

    # close dataconnection success
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_create_delete_success(self, mock_delete):
        response = disconnect(
            "url_delete_data_connection_success", DataConnectionId("data_connection_id")
        )
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # redirect success
    @mock.patch("requests.put", side_effect=mocked_requests_put)
    def test_create_redirect_success(self, mock_delete):
        response = redirect(
            "url_redirect_success",
            DataId("data_id"),
            DataConnectionId("data_connection_id"),
            {},
        )
        self.assertEqual(response.is_ok(), True)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "peer_api", TestCreatePeertResp)
