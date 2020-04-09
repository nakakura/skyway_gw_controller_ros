#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType
import simplejson as json
from common import post, get, put, delete, ApiResponse
from peer import PeerInfo


class MediaId:
    def __init__(self, media_id):
        # type: (str) -> MediaId
        self.__media_id = media_id

    def id(self):
        # type: () -> str
        return self.__media_id


class MediaConnectionId:
    def __init__(self, media_connection_id):
        # type: (str) -> MediaConnectionId
        self.__media_connection_id = media_connection_id

    def id(self):
        # type: () -> str
        return self.__media_connection_id


class RtcpId:
    def __init__(self, rtcp_id):
        # type: (str) -> RtcpId
        self.__rtcp_id = rtcp_id

    def id(self):
        # type: () -> str
        return self.__rtcp_id


class CallOption:
    def __init__(self, peer_info, target_id, constraints, redirect_params):
        # type: (PeerInfo, str, dict, dict) -> CallOption
        self.__peer_id = peer_info.id()
        self.__token = peer_info.token()
        self.__target_id = target_id
        self.__constraints = constraints
        self.__redirect_params = redirect_params

    def json(self):
        # type: () -> dict
        return {
            "peer_id": self.__peer_id,
            "token": self.__token,
            "constraints": self.__constraints,
            "redirect_params": self.__redirect_params,
        }


class AnswerOption:
    def __init__(self, constraints, redirect_params):
        # type: (dict, dict) -> AnswerOption
        self.__constraints = constraints
        self.__redirect_params = redirect_params

    def json(self):
        # type: () -> dict
        return {
            "constraints": self.__constraints,
            "redirect_params": self.__redirect_params,
        }


# This method call POST /media API to open a MediaSocket
# http://35.200.46.204/#/3.media/media
def create_media(url, is_video):
    # type: (str, bool) -> ApiResponse
    return post("{}/media".format(url), {"is_video": is_video}, 201)


# This method call DELETE /media/{media_id} API to close a MediaSocket
# http://35.200.46.204/#/3.media/streams_delete
def delete_media(url, media_id):
    # type: (str, MediaId) -> ApiResponse
    return delete("{}/media/{}".format(url, media_id.id()), 204)


# This method call POST /media/rtcp API to open a RtcpSocket
# http://35.200.46.204/#/3.media/media_rtcp_create
def create_rtcp(url):
    # type: (str) -> ApiResponse
    return post("{}/media/rtcp".format(url), {}, 201)


# This method call POST /media/rtcp API to open a RtcpSocket
# http://35.200.46.204/#/3.media/media_rtcp_create
def delete_rtcp(url, rtcp_id):
    # type: (str, RtcpId) -> ApiResponse
    return delete("{}/media/rtcp/{}".format(url, rtcp_id.id()), 204)


# This method call POST /media/connections API to establish P2P link
# http://35.200.46.204/#/3.media/media_connection_create
def call(url, call_option):
    # type: (str, CallOption) -> ApiResponse
    return post("{}/media/connections".format(url), call_option.json(), 202)


# This method call DELETE /media/connections API to close P2P link
# http://35.200.46.204/#/3.media/media_connection_close
def disconnect(url, media_connection_id):
    # type: (str, MediaConnectionId) -> ApiResponse
    return delete("{}/media/connections/{}".format(url, media_connection_id.id()), 204)


# This method call POST /media/connections/{media_connection_id}/answer API to accept P2P link establishment
# http://35.200.46.204/#/3.media/media_connection_close
def answer(url, media_connection_id, answer_option):
    # type: (str, MediaConnectionId, AnswerOption) -> ApiResponse
    return post(
        "{}/media/connections/{}/answer".format(url, media_connection_id.id()),
        answer_option.json(),
        202,
    )
