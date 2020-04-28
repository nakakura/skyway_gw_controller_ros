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
from service.listen_peer_events import *
from api.rest import ApiResponse
from api import peer


class TestListenPeerEvent(unittest.TestCase):
    peer_info = {}

    def setUp(self):
        self.peer_info = peer.PeerInfo("peer_id", "token")

    def tearDown(self):
        pass

    def test_listen_event_open(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()

        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {"event": "OPEN", "params": {"peer_id": "hoge", "token": "pt-test",},},
                {},
            ),
        ) as mock_listen_events:
            listen_peer_events(
                self.peer_info, control_queue, media_event_queue, data_event_queue
            )
            self.assertEqual(mock_listen_events.call_count, 2)

    def test_listen_event_close(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        media_event_queue.put = MagicMock()
        data_event_queue = multiprocessing.Queue()
        data_event_queue.put = MagicMock()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {"event": "CLOSE", "params": {"peer_id": "hoge", "token": "pt-test"}},
                {},
            ),
        ) as mock_listen_events:
            listen_peer_events(
                self.peer_info, control_queue, media_event_queue, data_event_queue
            )
            self.assertEqual(mock_listen_events.call_count, 1)
            self.assertEqual(media_event_queue.put.call_count, 1)
            self.assertEqual(data_event_queue.put.call_count, 1)

    def test_listen_event_connection(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()
        data_event_queue.put = MagicMock()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {
                    "event": "CONNECTION",
                    "data_params": {"data_connection_id": "da-test"},
                },
                {},
            ),
        ) as mock_listen_events:
            listen_peer_events(
                self.peer_info, control_queue, media_event_queue, data_event_queue
            )
            self.assertEqual(mock_listen_events.call_count, 1)
            self.assertEqual(data_event_queue.put.call_count, 1)
            self.assertEqual(
                data_event_queue.put.call_args[0][0],
                {"type": "CONNECTION", "data_connection_id": "da-test"},
            )

    def test_listen_event_request_error(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            side_effect=requests.exceptions.RequestException("error"),
        ) as mock_listen_events:
            with self.assertRaises(requests.exceptions.RequestException):
                listen_peer_events(
                    self.peer_info, control_queue, media_event_queue, data_event_queue
                )

    def test_listen_event_invalid_code_400(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {}, {"url": "error_url", "error": "400 Bad Request", "status": 400},
            ),
        ) as mock_listen_events:
            with self.assertRaises(MyError):
                listen_peer_events(
                    self.peer_info, control_queue, media_event_queue, data_event_queue
                )

    def test_listen_event_request_timeout(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {}, {"url": "error_url", "error": "408 Timeout", "status": 408},
            ),
        ) as mock_listen_events:
            listen_peer_events(
                self.peer_info, control_queue, media_event_queue, data_event_queue
            )
            # listen_peer_events ignores 408 Timeout and keep polling
            self.assertEqual(mock_listen_events.call_count, 2)

    def test_listen_event_call(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()
        media_event_queue.put = MagicMock()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {"event": "CALL", "call_params": {"media_connection_id": "mc-test"},},
                {},
            ),
        ) as mock_listen_events:
            listen_peer_events(
                self.peer_info, control_queue, media_event_queue, data_event_queue
            )
            self.assertEqual(mock_listen_events.call_count, 1)
            self.assertEqual(media_event_queue.put.call_count, 1)
            self.assertEqual(
                media_event_queue.put.call_args[0][0],
                {"media_connection_id": "mc-test", "type": "CALL"},
            )

    def test_listen_event_error(self):
        control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()
        media_event_queue.put = MagicMock()

        # FEED 2 PASS message, but listen_peer_events will access to listen_event only once,
        # because it returns CLOSE event.
        control_queue.put({"type": "PASS"})
        control_queue.put({"type": "APP_CLOSING"})

        with patch(
            "scripts.api.peer.listen_event",
            return_value=ApiResponse(
                {"event": "ERROR", "error_message": "BROWSER_INCOMPATIBLE"}, {},
            ),
        ):
            with self.assertRaises(MyError):
                listen_peer_events(
                    self.peer_info, control_queue, media_event_queue, data_event_queue
                )


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "listen_peer_event", TestListenPeerEvent)
