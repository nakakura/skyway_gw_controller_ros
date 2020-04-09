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

    rospy.logerr(args[0])
    if args[0] == "url_create_media/media":
        return MockResponse(args[0], {}, 201)
    elif args[0] == "url_create_rtcp/media/rtcp":
        return MockResponse(args[0], {}, 201)
    elif args[0] == "url_call_success/media/connections":
        return MockResponse(args[0], {}, 202)

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
    elif args[0] == "url_delete_rtcp/media/rtcp/rtcp_id":
        return MockResponse(args[0], {}, 204)
    elif args[0] == "url_disconnect_success/media/connections/media_connection_id":
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

    # close RtcpSocket
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_delete_rtcp_success(self, mock_post):
        response = delete_rtcp("url_delete_rtcp", RtcpId("rtcp_id"))
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # call success
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_call_success(self, mock_post):
        constraints = {
            "video": True,
            "videoReceiveEnabled": True,
            "audio": True,
            "audioReceiveEnabled": True,
            "video_params": {
                "band_width": 0,
                "codec": "H264",
                "media_id": "vi-test",
                "rtcp_id": "rc-test1",
                "payload_type": 100,
                "sampling_rate": 90000,
            },
            "audio_params": {
                "band_width": 0,
                "codec": "OPUS",
                "media_id": "au-test",
                "rtcp_id": "rc-test2",
                "payload_type": 111,
                "sampling_rate": 48000,
            },
        }
        redirects = {
            "video": True,
            "videoReceiveEnabled": True,
            "audio": True,
            "audioReceiveEnabled": True,
            "video_params": {
                "band_width": 0,
                "codec": "H264",
                "media_id": "vi-test2",
                "rtcp_id": "rc-test3",
                "payload_type": 100,
                "sampling_rate": 90000,
            },
            "audio_params": {
                "band_width": 0,
                "codec": "OPUS",
                "media_id": "au-test2",
                "rtcp_id": "rc-test3",
                "payload_type": 111,
                "sampling_rate": 48000,
            },
        }
        option = CallOption(
            PeerInfo("peer_id", "token"), "target_id", constraints, redirects
        )
        response = call("url_call_success", option)
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # disconnect success
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_delete_rtcp_success(self, mock_post):
        response = disconnect(
            "url_disconnect_success", MediaConnectionId("media_connection_id")
        )
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "media_api", TestMediaApi)
