#!/usr/bin/env python
# -*- coding: utf-8 -*-
PKG = "skyway"
import logging
import rospy
import roslib
import requests
import multiprocessing

roslib.load_manifest(PKG)

import sys
import unittest
from os import path
from mock import *

sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.abspath(__file__)))) + "/scripts"
)
import const
from service.data_redirect import *
from api.rest import ApiResponse
import api.data

# def status(data_connection_id):
# @mock.patch("data.status", return_value=mocked_requests_get)
class TestDataRedirect(unittest.TestCase):
    const.URL = "url_status_success"

    config = [
        {
            "name": "data",
            "codec": "VP8",
            "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
        },
        {"name": "data2", "codec": "H264"},
    ]

    def test_check_status_check_pass(self):
        response = ApiResponse(
            {
                "remote_id": "ID_BAR",
                "buffersize": 0,
                "label": "string",
                "metadata": "data",
                "open": True,
                "reliable": True,
                "serialization": "BINARY_UTF8",
                "type": "DATA",
            },
            {},
        )

        with patch("scripts.api.data.status", return_value=response) as mock_status:
            with patch(
                "scripts.api.data.disconnect", return_value={}
            ) as mock_disconnect:
                param = load_data_connection_config(
                    self.config, data.DataConnectionId("dc_test")
                )
                self.assertEqual(mock_status.call_count, 1)
                self.assertEqual(mock_status.call_args[0][0].id(), "dc_test")
                self.assertFalse(mock_disconnect.called, "disconnect called")
                self.assertEqual(
                    param,
                    {
                        "name": "data",
                        "codec": "VP8",
                        "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                    },
                )

    def test_check_status_check_fail(self):
        response = ApiResponse(
            {
                "remote_id": "ID_BAR",
                "buffersize": 0,
                "label": "string",
                "metadata": "invalid_data",
                "open": True,
                "reliable": True,
                "serialization": "BINARY_UTF8",
                "type": "DATA",
            },
            {},
        )

        with patch("scripts.api.data.status", return_value=response) as mock_status:
            with patch(
                "scripts.api.data.disconnect", return_value={}
            ) as mock_disconnect:
                with self.assertRaises(MyError):
                    _param = load_data_connection_config(
                        self.config, data.DataConnectionId("dc_test")
                    )
                self.assertEqual(mock_status.call_count, 1)
                self.assertEqual(mock_status.call_args[0][0].id(), "dc_test")
                self.assertEqual(mock_disconnect.call_args[0][0].id(), "dc_test")

    def test_check_status_check_status_method_raise_error(self):
        with patch(
            "scripts.api.data.status",
            side_effect=requests.exceptions.RequestException("error"),
        ) as mock_status:
            with patch(
                "scripts.api.data.disconnect", return_value={}
            ) as mock_disconnect:
                with self.assertRaises(requests.exceptions.RequestException):
                    (_data_connection_id, _param) = load_data_connection_config(
                        self.config, data.DataConnectionId("dc_test")
                    )
                self.assertEqual(mock_status.call_count, 1)
                self.assertEqual(mock_status.call_args[0][0].id(), "dc_test")
                self.assertFalse(mock_disconnect.called)

    def test_check_status_check_fail_delete_method_raise_error(self):
        response = ApiResponse(
            {
                "remote_id": "ID_BAR",
                "buffersize": 0,
                "label": "string",
                "metadata": "invalid_data",
                "open": True,
                "reliable": True,
                "serialization": "BINARY_UTF8",
                "type": "DATA",
            },
            {},
        )

        with patch("scripts.api.data.status", return_value=response) as mock_status:
            with patch(
                "scripts.api.data.disconnect",
                side_effect=requests.exceptions.RequestException("error"),
            ) as mock_disconnect:
                with self.assertRaises(requests.exceptions.RequestException):
                    (_param) = load_data_connection_config(
                        self.config, data.DataConnectionId("dc_test")
                    )
                self.assertEqual(mock_status.call_count, 1)
                self.assertEqual(mock_status.call_args[0][0].id(), "dc_test")
                self.assertEqual(mock_disconnect.call_count, 1)
                self.assertEqual(mock_disconnect.call_args[0][0].id(), "dc_test")

    def test_create_redirect_params(self):
        response = ApiResponse(
            {"data_id": "da-test", "port": 10001, "ip_v4": "127.0.0.1"}, {},
        )

        with patch("scripts.api.data.create_data", return_value=response) as mock:
            (redirect_params, data_socket_info) = create_redirect_params({})
            self.assertEqual(
                redirect_params,
                {"redirect_params": {}, "feed_params": {"data_id": "da-test"}},
            )
            self.assertEqual(
                data_socket_info,
                {"ip_v4": "127.0.0.1", "port": 10001, "data_id": "da-test"},
            )
            self.assertEqual(mock.call_count, 1)

    def test_create_redirect_params_with_redirect_params(self):
        response = ApiResponse(
            {"data_id": "da-test", "port": 10001, "ip_v4": "127.0.0.1"}, {},
        )

        with patch("scripts.api.data.create_data", return_value=response) as mock:
            (redirect_params, data_socket_info) = create_redirect_params(
                {
                    "name": "data",
                    "codec": "VP8",
                    "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                }
            )
            self.assertEqual(
                redirect_params,
                {
                    "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                    "feed_params": {"data_id": "da-test"},
                },
            )
            self.assertEqual(
                data_socket_info,
                {"ip_v4": "127.0.0.1", "port": 10001, "data_id": "da-test"},
            )
            self.assertEqual(mock.call_count, 1)

    def test_create_redirect_params_error(self):
        with patch(
            "scripts.api.data.create_data",
            side_effect=requests.exceptions.RequestException("error"),
        ) as mock:
            with self.assertRaises(requests.exceptions.RequestException):
                (_redirect_params, _data_socket_info) = create_redirect_params(
                    {
                        "name": "data",
                        "codec": "VP8",
                        "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                    }
                )

    def test_on_connect_success(self):
        with patch(
            "service.data_redirect.load_data_connection_config",
            return_value={
                "name": "data",
                "codec": "VP8",
                "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
            },
        ) as mock_data_redirect:
            with patch(
                "service.data_redirect.create_redirect_params",
                return_value=(
                    {
                        "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                        "feed_params": {"data_id": "da-test"},
                    },
                    {"ip_v4": "127.0.0.1", "port": 10001, "data_id": "da-test"},
                ),
            ) as mock_create_redirect_params:
                with patch(
                    "scripts.api.data.redirect",
                    return_value=ApiResponse(
                        {
                            "command_type": "DATA_CONNECTION_PUT",
                            "data_id": "da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
                        },
                        {},
                    ),
                ) as mock_redirect_api:
                    queue = multiprocessing.Queue()
                    queue.put = MagicMock()
                    config = on_connect(
                        queue, self.config, data.DataConnectionId("dc_test")
                    )
                    # these three mocks should be called
                    self.assertTrue(mock_data_redirect.called)
                    self.assertTrue(mock_create_redirect_params.called)
                    self.assertTrue(mock_redirect_api.called)
                    self.assertEqual(config, [{"name": "data2", "codec": "H264"}])
                    self.assertTrue(queue.put.called)
                    # queue recv connection event, which will be sent to ROS subscriber
                    self.assertEqual(
                        queue.put.call_args[0][0],
                        {
                            "type": "CONNECTION",
                            "value": {
                                "data": {
                                    "command_type": "DATA_CONNECTION_PUT",
                                    "data_id": "da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
                                },
                                "redirect_params": {
                                    "feed_params": {"data_id": "da-test"},
                                    "redirect_params": {
                                        "ip_v4": "127.0.0.1",
                                        "port": 8000,
                                    },
                                },
                            },
                        },
                    )

    def test_on_connect_load_connection_config_fail(self):
        with patch(
            "service.data_redirect.load_data_connection_config",
            side_effect=MyError("error"),
        ) as mock_data_redirect:
            with patch(
                "service.data_redirect.create_redirect_params",
                return_value=(
                    {
                        "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                        "feed_params": {"data_id": "da-test"},
                    },
                    {"ip_v4": "127.0.0.1", "port": 10001, "data_id": "da-test"},
                ),
            ) as mock_create_redirect_params:
                with patch(
                    "scripts.api.data.redirect",
                    side_effect=requests.exceptions.RequestException("error"),
                ) as mock_redirect_api:
                    queue = multiprocessing.Queue()
                    queue.put = MagicMock()
                    config = on_connect(
                        queue, self.config, data.DataConnectionId("dc_test")
                    )
                    # these three mocks should be called
                    self.assertTrue(mock_data_redirect.called)
                    self.assertFalse(mock_create_redirect_params.called)
                    self.assertFalse(mock_redirect_api.called)
                    self.assertEqual(config, self.config)
                    # Queue doesn't recv any data
                    self.assertFalse(queue.put.called)

    # TestCase for WebRTC gw returns invalid data
    # as a response of PUT /data/connections/{data_connection_id}
    # This case means WebRTC Gateway has some bug.
    # So just raise Error
    def test_on_connect_redirect_response_400(self):
        with patch(
            "service.data_redirect.load_data_connection_config",
            return_value={
                "name": "data",
                "codec": "VP8",
                "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
            },
        ) as mock_data_redirect:
            with patch(
                "service.data_redirect.create_redirect_params",
                return_value=(
                    {
                        "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                        "feed_params": {"data_id": "da-test"},
                    },
                    {"ip_v4": "127.0.0.1", "port": 10001, "data_id": "da-test"},
                ),
            ) as mock_create_redirect_params:
                with patch(
                    "scripts.api.data.redirect",
                    return_value=ApiResponse(
                        {},
                        {
                            "command_type": "DATA_CONNECTION_DELETE",
                            "params": {
                                "errors": [
                                    {
                                        "field": "peer_id",
                                        "message": "peer_id field is not specified",
                                    }
                                ]
                            },
                        },
                    ),
                ) as mock_redirect_api:
                    queue = multiprocessing.Queue()
                    queue.put = MagicMock()
                    with self.assertRaises(MyError):
                        _config = on_connect(
                            queue, self.config, data.DataConnectionId("dc_test")
                        )
                    # these three mocks should be called
                    self.assertTrue(mock_data_redirect.called)
                    self.assertTrue(mock_create_redirect_params.called)
                    self.assertTrue(mock_redirect_api.called)
                    # Queue doesn't recv any data
                    self.assertFalse(queue.put.called)

    # TestCase for WebRTC gw crashed after on_connect start processing
    def test_on_connect_redirect_fail(self):
        with patch(
            "service.data_redirect.load_data_connection_config",
            return_value={
                "name": "data",
                "codec": "VP8",
                "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
            },
        ) as mock_data_redirect:
            with patch(
                "service.data_redirect.create_redirect_params",
                return_value=(
                    {
                        "redirect_params": {"ip_v4": "127.0.0.1", "port": 8000},
                        "feed_params": {"data_id": "da-test"},
                    },
                    {"ip_v4": "127.0.0.1", "port": 10001, "data_id": "da-test"},
                ),
            ) as mock_create_redirect_params:
                with patch(
                    "scripts.api.data.redirect",
                    side_effect=requests.exceptions.RequestException("error"),
                ) as mock_redirect_api:
                    queue = multiprocessing.Queue()
                    queue.put = MagicMock()
                    with self.assertRaises(requests.exceptions.RequestException):
                        config = on_connect(
                            queue, self.config, data.DataConnectionId("dc_test")
                        )
                        self.assertTrue(False, "this method should not be called")
                    # these three mocks should be called
                    self.assertTrue(mock_data_redirect.called)
                    self.assertTrue(mock_create_redirect_params.called)
                    self.assertTrue(mock_redirect_api.called)
                    # Queue doesn't recv any data
                    self.assertFalse(queue.put.called)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_redirect", TestDataRedirect)
