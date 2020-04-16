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
import multiprocessing

sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.abspath(__file__)))) + "/scripts"
)
from scripts.api.data import *
import const


class MockResponse:
    def __init__(self, url, json_data, status_code):
        self.url = url
        self.json_data = json_data
        self.status_code = status_code

    def json(self):
        return self.json_data


def mocked_requests_get(*args, **kwargs):
    if args[0] == "url_create_data_success/data":
        return MockResponse(args[0], {}, 200)
    elif args[0] == "url_event_success/data/connections/data_connection_id/events":
        return MockResponse(args[0], {}, 200)
    elif args[0] == "url_status_success/data/connections/data_connection_id/status":
        return MockResponse(args[0], {}, 200)
    elif args[0] == "test_onevent_connect_not_intend/data/connections/dc_test/status":
        return MockResponse(
            args[0],
            {
                u"type": u"DATA",
                u"reliable": False,
                u"remote_id": u"data_caller",
                u"label": u"c_lfcag5dg1ufo47c6v6awi2j4i",
                u"serialization": u"BINARY",
                u"buffersize": 0,
                u"open": False,
                u"metadata": u"not intend data",
            },
            200,
        )

    return MockResponse(args[0], {}, 410)


def mocked_requests_delete(*args, **kwargs):
    if args[0] == "url_delete_data_success/data/data_id":
        return MockResponse(args[0], {}, 204)
    elif (
        args[0]
        == "url_delete_data_connection_success/data/connections/data_connection_id"
    ):
        return MockResponse(args[0], {}, 204)
    elif args[0] == "test_onevent_connect_not_intend/data/connections/dc_test":
        return MockResponse(args[0], {}, 204)

    return MockResponse(args[0], {}, 410)


def mocked_requests_post(*args, **kwargs):
    if args[0] == "url_connections_success/data/connections":
        return MockResponse(args[0], kwargs["json"], 202)

    return MockResponse(args[0], {}, 410)


def mocked_requests_put(*args, **kwargs):
    if args[0] == "url_redirect_success/data/connections/data_connection_id":
        return MockResponse(args[0], kwargs["json"], 200)

    return MockResponse(args[0], {}, 410)


class TestDataApi(unittest.TestCase):
    # open dataport
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_create_data_success(self, mock_post):
        const.URL = "url_create_data_success"
        response = create_data()
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # close dataport
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_delete_data_success(self, mock_delete):
        const.URL = "url_delete_data_success"
        response = delete_data(DataId("data_id"))
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # establish dataconnection success
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_connect_success(self, mock_post):
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
        const.URL = "url_connections_success"
        response = connect(options)
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
    def test_disconnect_success(self, mock_delete):
        const.URL = "url_delete_data_connection_success"
        response = disconnect(DataConnectionId("data_connection_id"))
        self.assertEqual(response.json(), {})
        self.assertEqual(response.is_ok(), True)

    # redirect success
    @mock.patch("requests.put", side_effect=mocked_requests_put)
    def test_redirect_success(self, mock_delete):
        const.URL = "url_redirect_success"
        response = redirect(
            DataId("data_id"), DataConnectionId("data_connection_id"), {},
        )
        self.assertEqual(response.is_ok(), True)

    # events success
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_event_success(self, mock_get):
        const.URL = "url_event_success"
        response = events(DataConnectionId("data_connection_id"))
        self.assertEqual(response.is_ok(), True)

    # status success
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_status_success(self, mock_get):
        const.URL = "url_status_success"
        response = status(DataConnectionId("data_connection_id"))
        self.assertEqual(response.is_ok(), True)

    # events connect
    # Anticipated Connection
    # def test_onevent_timeout(self):
    #     with self.assertRaises(Exception):
    #         queue = multiprocessing.Queue()
    #         on_events({"name": "data"}, queue)

    # events connect
    # Just Close the loop
    def test_onevent_close(self):
        config = [{"name": "data"}, {"name": "data2"}]
        queue = multiprocessing.Queue()
        queue.put({"type": "APP_CLOSING"})
        mock_lib = mock.MagicMock()
        with patch("scripts.api.data.on_connect", side_effect=mock_lib):
            on_events(config, queue)
            self.assertTrue(not mock_lib.called, "on_connect is called")

    # events connect
    # Just Close the loop
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_onevent_connect_status_check_success(self, mock_get):
        const.URL = "test_onevent_connect_not_intend"
        queue = multiprocessing.Queue()
        queue.put({"type": "CONNECTION", "data_connection_id": "dc_test"})
        queue.put({"type": "APP_CLOSING"})
        mock_lib = mock.MagicMock()
        with patch("scripts.api.data.on_connect", side_effect=mock_lib):
            config = [{"name": "data"}, {"name": "data2"}]
            on_events(config, queue)
            self.assertTrue(mock_lib.called, "on_connect is not called")
            self.assertEqual(
                mock_lib.call_args[0][0], [{"name": "data"}, {"name": "data2"}]
            )
            self.assertEqual(
                mock_lib.call_args[0][1].id(), DataConnectionId("dc_test").id()
            )

    # on_connect
    # load redirect information from config using metadata as a key
    # If it is not an intended connection, disconnect it
    def test_onconnect_status_check_fail(self):
        mock_lib_disconnect = mock.MagicMock()
        mock_lib_redirect = mock.MagicMock()
        with patch("scripts.api.data.disconnect", side_effect=mock_lib_disconnect):
            with patch("scripts.api.data.redirect", side_effect=mock_lib_redirect):
                config = [{"name": "data"}, {"name": "data2"}]
                data_connection_id = DataConnectionId("dc_test")
                on_connect(config, data_connection_id, "data3")
                self.assertFalse(mock_lib_redirect.called, "redirect is called")
                self.assertTrue(mock_lib_disconnect.called, "disconnect is not called")
                self.assertEqual(
                    mock_lib_disconnect.call_args[0][0], data_connection_id
                )

    # on_connect
    # load redirect information from config using metadata as a key
    def test_onconnect_status_check_success_no_redirect_params(self):
        mock_lib_disconnect = mock.MagicMock()
        mock_lib_redirect = mock.MagicMock()
        ret_value = ApiResponse(
            {"data_id": "da-test", "port": 10001, "ip_v4": "127.0.0.1"}, {}
        )
        with patch("scripts.api.data.disconnect", side_effect=mock_lib_disconnect):
            with patch("scripts.api.data.redirect", side_effect=mock_lib_redirect):
                with patch("scripts.api.data.create_data", return_value=ret_value):
                    config = [
                        {"name": "data", "codec": "VP8"},
                        {"name": "data2", "codec": "H264"},
                    ]
                    data_connection_id = DataConnectionId("dc_test")
                    on_connect(config, data_connection_id, "data2")
                    self.assertFalse(mock_lib_disconnect.called, "disconnect is called")
                    self.assertTrue(mock_lib_redirect.called, "redirect is not called")
                    self.assertEqual(
                        mock_lib_redirect.call_args[0][0].id(), DataId("da-test").id()
                    )
                    self.assertEqual(
                        mock_lib_redirect.call_args[0][1].id(), data_connection_id.id()
                    )
                    self.assertEqual(
                        mock_lib_redirect.call_args[0][2], {},
                    )

    # on_connect
    # load redirect information from config using metadata as a key
    def test_onconnect_status_check_success_with_redirect_params(self):
        mock_lib_disconnect = mock.MagicMock()
        mock_lib_redirect = mock.MagicMock()
        ret_value = ApiResponse(
            {"data_id": "da-test", "port": 10001, "ip_v4": "127.0.0.1"}, {}
        )
        with patch("scripts.api.data.disconnect", side_effect=mock_lib_disconnect):
            with patch("scripts.api.data.redirect", side_effect=mock_lib_redirect):
                with patch("scripts.api.data.create_data", return_value=ret_value):
                    config = [
                        {
                            "name": "data",
                            "codec": "VP8",
                            "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                        },
                        {"name": "data2", "codec": "H264"},
                    ]
                    data_connection_id = DataConnectionId("dc_test")
                    on_connect(config, data_connection_id, "data")
                    self.assertFalse(mock_lib_disconnect.called, "disconnect is called")
                    self.assertTrue(mock_lib_redirect.called, "redirect is not called")
                    self.assertEqual(
                        mock_lib_redirect.call_args[0][0].id(), DataId("da-test").id()
                    )
                    self.assertEqual(
                        mock_lib_redirect.call_args[0][1].id(), data_connection_id.id()
                    )
                    self.assertEqual(
                        mock_lib_redirect.call_args[0][2],
                        {"ip_v4": "127.0.0.1", "port": 8000},
                    )


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestDataApi)
