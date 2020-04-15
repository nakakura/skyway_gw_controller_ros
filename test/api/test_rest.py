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
from scripts.api.rest import *


def mocked_requests_post(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_201":
        if kwargs["json"] == {"param": "valid"}:
            return MockResponse(args[0], {"key1": "value1"}, 201)
        else:
            return MockResponse(args[0], {}, 403)
    elif args[0] == "url_400":
        return MockResponse(args[0], {}, 400)
    elif args[0] == "url_403":
        return MockResponse(args[0], {}, 403)
    elif args[0] == "url_404":
        return MockResponse(args[0], {}, 404)
    elif args[0] == "url_405":
        return MockResponse(args[0], {}, 405)
    elif args[0] == "url_406":
        return MockResponse(args[0], {}, 406)
    elif args[0] == "url_408":
        return MockResponse(args[0], {}, 408)

    # invalid status code
    return MockResponse(args[0], {}, 999)


def mocked_requests_get(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_200":
        return MockResponse(args[0], {"response": "valid"}, 200)
    elif args[0] == "url_400":
        return MockResponse(args[0], {}, 400)
    elif args[0] == "url_403":
        return MockResponse(args[0], {}, 403)
    elif args[0] == "url_404":
        return MockResponse(args[0], {}, 404)
    elif args[0] == "url_405":
        return MockResponse(args[0], {}, 405)
    elif args[0] == "url_406":
        return MockResponse(args[0], {}, 406)
    elif args[0] == "url_408":
        return MockResponse(args[0], {}, 408)

    # invalid status code
    return MockResponse({}, 999)


def mocked_requests_put(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_200":
        if kwargs["json"] == {"param": "valid"}:
            return MockResponse(args[0], {"response": "valid"}, 200)
        else:
            return MockResponse(args[0], {}, 403)
    elif args[0] == "url_400":
        return MockResponse(args[0], {}, 400)
    elif args[0] == "url_403":
        return MockResponse(args[0], {}, 403)
    elif args[0] == "url_404":
        return MockResponse(args[0], {}, 404)
    elif args[0] == "url_405":
        return MockResponse(args[0], {}, 405)
    elif args[0] == "url_406":
        return MockResponse(args[0], {}, 406)
    elif args[0] == "url_408":
        return MockResponse(args[0], {}, 408)

    # invalid status code
    return MockResponse({}, 999)


def mocked_requests_delete(*args, **kwargs):
    class MockResponse:
        def __init__(self, url, json_data, status_code):
            self.json_data = json_data
            self.status_code = status_code
            self.url = url

        def json(self):
            return self.json_data

    if args[0] == "url_204":
        return MockResponse(args[0], {}, 204)
    elif args[0] == "url_400":
        return MockResponse(args[0], {}, 400)
    elif args[0] == "url_403":
        return MockResponse(args[0], {}, 403)
    elif args[0] == "url_404":
        return MockResponse(args[0], {}, 404)
    elif args[0] == "url_405":
        return MockResponse(args[0], {}, 405)
    elif args[0] == "url_406":
        return MockResponse(args[0], {}, 406)
    elif args[0] == "url_408":
        return MockResponse(args[0], {}, 408)

    # invalid status code
    return MockResponse({}, 999)


class TestCommonRest(unittest.TestCase):
    # -------------------- POST --------------------
    # success case
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_success(self, mock_post):
        response = post("url_201", {"param": "valid"}, 201)
        self.assertEqual(response.json(), {"key1": "value1"})
        self.assertEqual(response.is_ok(), True)

    # return 400
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_400(self, mock_post):
        response = post("url_400", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(),
            {"url": "url_400", "status": 400, "error": "400 Bad Request"},
        )
        self.assertEqual(response.is_ok(), False)

    # return 403
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_403(self, mock_post):
        response = post("url_403", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(), {"url": "url_403", "status": 403, "error": "403 Forbidden"}
        )
        self.assertEqual(response.is_ok(), False)

    # return 404
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_404(self, mock_post):
        response = post("url_404", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(), {"url": "url_404", "status": 404, "error": "404 Not Found"}
        )
        self.assertEqual(response.is_ok(), False)

    # return 405
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_405(self, mock_post):
        response = post("url_405", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(),
            {"url": "url_405", "status": 405, "error": "405 Method Not Allowed"},
        )
        self.assertEqual(response.is_ok(), False)

    # return 406
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_406(self, mock_post):
        response = post("url_406", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(),
            {"url": "url_406", "status": 406, "error": "406 Not Acceptable"},
        )
        self.assertEqual(response.is_ok(), False)

    # return 408
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_408(self, mock_post):
        response = post("url_408", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(),
            {"url": "url_408", "status": 408, "error": "408 Request Timeout"},
        )
        self.assertEqual(response.is_ok(), False)

    # return unknown code
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_410(self, mock_post):
        response = post("url_410", {"param": "valid"}, 201)
        self.assertEqual(
            response.err(),
            {"url": "url_410", "status": 999, "error": "Unexpected Status Code"},
        )
        self.assertEqual(response.is_ok(), False)

    # wrong post parameters
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_with_wrong_params(self, mock_post):
        response = post("url_201", {"param": "invalid"}, 201)
        self.assertEqual(
            response.err(), {"url": "url_201", "status": 403, "error": "403 Forbidden"}
        )
        self.assertEqual(response.is_ok(), False)

    # wrong post parameters
    @mock.patch("requests.post", side_effect=mocked_requests_post)
    def test_post_wait_wrong_statuscode(self, mock_post):
        response = post("url_201", {"param": "valid"}, 202)
        self.assertEqual(
            response.err(),
            {"url": "url_201", "status": 201, "error": "Unexpected Status Code"},
        )
        self.assertEqual(response.is_ok(), False)

    # -------------------- GET --------------------
    # get success
    @mock.patch("requests.get", side_effect=mocked_requests_get)
    def test_get_success(self, mock_get):
        response = get("url_200", 200)
        self.assertEqual(response.json(), {"response": "valid"})
        self.assertEqual(response.is_ok(), True)

    # -------------------- DELETE --------------------
    # delete success
    @mock.patch("requests.delete", side_effect=mocked_requests_delete)
    def test_delete_success(self, mock_delete):
        response = delete("url_204", 204)
        self.assertEqual(response.is_ok(), True)

    # -------------------- PUT --------------------
    # put success
    @mock.patch("requests.put", side_effect=mocked_requests_put)
    def test_put_success(self, mock_delete):
        response = put("url_200", {"param": "valid"}, 200)
        self.assertEqual(response.json(), {"response": "valid"})
        self.assertEqual(response.is_ok(), True)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "api_common", TestCommonRest)
