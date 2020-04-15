#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType
import simplejson as json
from rest import post, get, put, delete, ApiResponse
from peer import PeerInfo
import const


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
def create_media(is_video):
    # type: (bool) -> ApiResponse
    return post("{}/media".format(const.URL), {"is_video": is_video}, 201)


# This method call DELETE /media/{media_id} API to close a MediaSocket
# http://35.200.46.204/#/3.media/streams_delete
def delete_media(media_id):
    # type: (MediaId) -> ApiResponse
    return delete("{}/media/{}".format(const.URL, media_id.id()), 204)


# This method call POST /media/rtcp API to open a RtcpSocket
# http://35.200.46.204/#/3.media/media_rtcp_create
def create_rtcp():
    # type: () -> ApiResponse
    return post("{}/media/rtcp".format(const.URL), {}, 201)


# This method call POST /media/rtcp API to open a RtcpSocket
# http://35.200.46.204/#/3.media/media_rtcp_create
def delete_rtcp(rtcp_id):
    # type: (RtcpId) -> ApiResponse
    return delete("{}/media/rtcp/{}".format(const.URL, rtcp_id.id()), 204)


# This method call POST /media/connections API to establish P2P link
# http://35.200.46.204/#/3.media/media_connection_create
def call(call_option):
    # type: (CallOption) -> ApiResponse
    return post("{}/media/connections".format(const.URL), call_option.json(), 202)


# This method call DELETE /media/connections API to close P2P link
# http://35.200.46.204/#/3.media/media_connection_close
def disconnect(media_connection_id):
    # type: (MediaConnectionId) -> ApiResponse
    return delete(
        "{}/media/connections/{}".format(const.URL, media_connection_id.id()), 204
    )


# This method call POST /media/connections/{media_connection_id}/answer API to accept P2P link establishment
# http://35.200.46.204/#/3.media/media_connection_close
def answer(media_connection_id, answer_option):
    # type: (MediaConnectionId, AnswerOption) -> ApiResponse
    return post(
        "{}/media/connections/{}/answer".format(const.URL, media_connection_id.id()),
        answer_option.json(),
        202,
    )


# This method call POST /media/connections/{media_connection_id}/pli API to send PLI message to neighbour
# http://35.200.46.204/#/3.media/media_connection_pli
def pli(media_connection_id, body):
    # type: (MediaConnectionId, dict) -> ApiResponse
    return post(
        "{}/media/connections/{}/pli".format(const.URL, media_connection_id.id()),
        body,
        201,
    )


# This method call POST /media/connections/{media_connection_id}/pli API to send PLI message to neighbour
# http://35.200.46.204/#/3.media/media_connection_pli
def event(media_connection_id):
    # type: (MediaConnectionId) -> ApiResponse
    return get(
        "{}/media/connections/{}/events".format(const.URL, media_connection_id.id()),
        200,
    )


# This method call GET /media/connections/{media_connection_id}/status API to get status of MediaChannel
# http://35.200.46.204/#/3.media/media_connection_status
def status(media_connection_id):
    # type: (MediaConnectionId) -> ApiResponse
    return get(
        "{}/media/connections/{}/status".format(const.URL, media_connection_id.id()),
        200,
    )
