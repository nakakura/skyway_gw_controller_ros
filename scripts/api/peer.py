#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType

from rest import post, get, put, delete, ApiResponse
import const


class PeerInfo:
    def __init__(self, peer_id, token):
        # type: (str, str) -> PeerInfo
        self.__peer_id = peer_id
        self.__token = token

    def id(self):
        # type: () -> str
        return self.__peer_id

    def token(self):
        # type: () -> str
        return self.__token


# This method call POST /peer API to create PeerObject
# http://35.200.46.204/#/1.peers/peer
def create_peer(key, domain, peer_info, turn):
    # type: (str, str, PeerInfo, bool) -> ApiResponse
    if (
        not isinstance(key, str)
        or not isinstance(domain, str)
        or not isinstance(peer_info, PeerInfo)
        or not isinstance(turn, bool)
    ):
        return ApiResponse({}, {"error": "invalid type error"})

    payload = {"key": key, "domain": domain, "peer_id": peer_info.id(), "turn": turn}
    return post("{}/peers".format(const.URL), payload, 201)


# This method call DELETE /peer API to delete PeerObject
# http://35.200.46.204/#/1.peers/peer_destroy
def delete_peer(peer_info):
    # type: (PeerInfo) -> ApiResponse
    if not isinstance(peer_info, PeerInfo):
        return ApiResponse({}, {"error": "invalid type error"})

    return delete(
        "{}/peers/{}?token={}".format(const.URL, peer_info.id(), peer_info.token()), 204
    )


# This method listen events of /peer API
# http://35.200.46.204/#/1.peers/peer_event
def listen_event(peer_info):
    # type: (PeerInfo) -> ApiResponse
    if not isinstance(peer_info, PeerInfo):
        return ApiResponse({}, {"error": "invalid type error"})

    return get(
        "{}/peers/{}/events?token={}".format(
            const.URL, peer_info.id(), peer_info.token()
        ),
        200,
    )


# This method call /peer/{peer_id}/status API to get the PeerObject's status
# http://35.200.46.204/#/1.peers/peer_status
def status(peer_info):
    # type: (PeerInfo) -> ApiResponse
    if not isinstance(peer_info, PeerInfo):
        return ApiResponse({}, {"error": "invalid type error"})

    return get(
        "{}/peers/{}/status?token={}".format(
            const.URL, peer_info.id(), peer_info.token()
        ),
        200,
    )
