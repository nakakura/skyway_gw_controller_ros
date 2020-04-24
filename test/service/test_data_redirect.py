#!/usr/bin/env python
# -*- coding: utf-8 -*-
PKG = "skyway"
import logging
import rospy
import roslib
import requests

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
                (data_connection_id, param) = load_dataconnection_config(
                    self.config, data.DataConnectionId("dc_test")
                )
                self.assertEqual(mock_status.call_count, 1)
                self.assertEqual(mock_status.call_args[0][0].id(), "dc_test")
                self.assertFalse(mock_disconnect.called, "disconnect called")
                self.assertEqual(data_connection_id.id(), "dc_test")
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
                    (_data_connection_id, _param) = load_dataconnection_config(
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
                    (_data_connection_id, _param) = load_dataconnection_config(
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
                    (_data_connection_id, _param) = load_dataconnection_config(
                        self.config, data.DataConnectionId("dc_test")
                    )
                self.assertEqual(mock_status.call_count, 1)
                self.assertEqual(mock_status.call_args[0][0].id(), "dc_test")
                self.assertEqual(mock_disconnect.call_count, 1)
                self.assertEqual(mock_disconnect.call_args[0][0].id(), "dc_test")


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_redirect", TestDataRedirect)
