#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType


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


# general method for post
def post(url, payload, expected_code):
    # type: (str, dict, int) -> ApiResponse
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
