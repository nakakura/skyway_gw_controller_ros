#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType


class PeerId:
    def __init__(self, peer_id):
        # type: (str) -> PeerId
        self.peer_id = peer_id

    def id(self):
        # type: () -> str
        return self.peer_id


class ApiResponse:
    def __init__(self, data, err):
        # type: (dict, dict) -> None

        self.data = data
        self.err = err

    def is_ok(self):
        # type: () -> bool
        return self.err == {}

    def json(self):
        # type: () -> dict
        return self.data


def post(url, payload, expected_code):
    # type: (str, dict, int) -> ApiResponse
    """docstring here"""

    headers = {"content-type": "application/json"}
    try:
        resp = requests.post(url, headers=headers, json=payload, timeout=(3.0, 1.0))

        if resp.status_code == expected_code:
            response = ApiResponse(resp.json(), {})
        elif resp.status_code == 400:
            response = ApiResponse({}, {"url": url, "error": "400 Bad Request"})
        elif resp.status_code == 403:
            response = ApiResponse({}, {"url": url, "error": "403 Forbidden"})
        elif resp.status_code == 404:
            response = ApiResponse({}, {"url": url, "error": "404 Not Found"})
        elif resp.status_code == 405:
            response = ApiResponse({}, {"url": url, "error": "405 Method Not Allowed"})
        elif resp.status_code == 406:
            response = ApiResponse({}, {"url": url, "error": "406 Not Acceptable"})
        elif resp.status_code == 408:
            response = ApiResponse({}, {"url": url, "error": "408 Request Timeout"})
        else:
            response = ApiResponse({}, {"url": url, "error": "Unexpected Status Code"})

        return response
    except requests.exceptions.RequestException as e:
        return ApiResponse({}, e)
    except Exception as e:
        raise e


# This method call POST /peer API to create PeerObject
# http://35.200.46.204/#/1.peers/peer
def create_peer(url, key, domain, peer_id, turn):
    # type: (str, str, str, PeerId, bool) -> ApiResponse
    """docstring here"""

    if (
        isinstance(url, str)
        and isinstance(key, str)
        and isinstance(domain, str)
        and isinstance(peer_id, PeerId)
        and isinstance(turn, bool)
    ):
        rospy.logerr("isinstance true")

    payload = {"key": key, "domain": domain, "peer_id": peer_id.id(), "turn": turn}
    return post(url + "/peers", payload, 201)
