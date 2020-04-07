#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import simplejson
import rospy
from typing import NewType


class ApiResponse:
    def __init__(self, data, err):
        # type: (dict, dict) -> None
        self.__data = data
        self.__err = err

    def is_ok(self):
        # type: () -> bool
        return self.__err == {}

    def json(self):
        # type: () -> dict
        return self.__data

    def err(self):
        # type: () -> dict
        return self.__err


def __parse_response(resp, expected_code):
    # type: (requests.Response, int) -> ApiResponse
    if resp.status_code == expected_code:
        try:
            response = ApiResponse(resp.json(), {})
        except simplejson.JSONDecodeError as e:
            response = ApiResponse({}, {})
    elif resp.status_code == 400:
        response = ApiResponse({}, {"url": resp.url, "error": "400 Bad Request"})
    elif resp.status_code == 403:
        response = ApiResponse({}, {"url": resp.url, "error": "403 Forbidden"})
    elif resp.status_code == 404:
        response = ApiResponse({}, {"url": resp.url, "error": "404 Not Found"})
    elif resp.status_code == 405:
        response = ApiResponse({}, {"url": resp.url, "error": "405 Method Not Allowed"})
    elif resp.status_code == 406:
        response = ApiResponse({}, {"url": resp.url, "error": "406 Not Acceptable"})
    elif resp.status_code == 408:
        response = ApiResponse({}, {"url": resp.url, "error": "408 Request Timeout"})
    else:
        response = ApiResponse({}, {"url": resp.url, "error": "Unexpected Status Code"})
    return response


# general method for post
def post(url, payload, expected_code):
    # type: (str, dict, int) -> ApiResponse
    headers = {"content-type": "application/json"}
    try:
        resp = requests.post(url, headers=headers, json=payload, timeout=(3.0, 1.0))
        return __parse_response(resp, expected_code)
    except requests.exceptions.RequestException as e:
        return ApiResponse({}, e)
    except Exception as e:
        raise e


# general method for get
def get(url, expected_code):
    # type: (str, int) -> ApiResponse
    headers = {"content-type": "application/json"}
    try:
        resp = requests.get(url, headers, timeout=(3.0, 1.0))
        return __parse_response(resp, expected_code)
    except requests.exceptions.RequestException as e:
        return ApiResponse({}, e)
    except Exception as e:
        raise e


# general method for put
def put(url, payload, expected_code):
    # type: (str, dict, int) -> ApiResponse
    headers = {"content-type": "application/json"}
    try:
        resp = requests.put(url, headers=headers, json=payload, timeout=(3.0, 1.0))
        return __parse_response(resp, expected_code)
    except requests.exceptions.RequestException as e:
        return ApiResponse({}, e)
    except Exception as e:
        raise e


# general method for delete
def delete(url, expected_code):
    # type: (str, int) -> ApiResponse
    headers = {"content-type": "application/json"}
    try:
        rospy.logerr(url)
        # SkyWay WebRTC Gateway's longpoll methods return timeout every 30secs.
        resp = requests.delete(url, headers=headers, timeout=(3.0, 30.0))
        return __parse_response(resp, expected_code)
    except requests.exceptions.RequestException as e:
        return ApiResponse({}, e)
    except Exception as e:
        raise e
