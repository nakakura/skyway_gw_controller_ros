#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType

from common import post, get, put, delete


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
def create_peer(url, key, domain, peer_info, turn):
    # type: (str, str, str, PeerInfo, bool) -> ApiResponse
    if (
        isinstance(url, str)
        and isinstance(key, str)
        and isinstance(domain, str)
        and isinstance(peer_info, PeerInfo)
        and isinstance(turn, bool)
    ):
        rospy.logerr("isinstance true")

    payload = {"key": key, "domain": domain, "peer_id": peer_info.id(), "turn": turn}
    return post(url + "/peers", payload, 201)


# This method call DELETE /peer API to delete PeerObject
# http://35.200.46.204/#/1.peers/peer_destroy
def delete_peer(url, peer_info):
    # type: (str, PeerInfo) -> ApiResponse
    if not isinstance(url, str) or not isinstance(peer_info, PeerInfo):
        rospy.logerr("isinstance true in delete_peer")

    return delete(url + "/peers/" + peer_info.id() + "?token=" + peer_info.token(), 204)
