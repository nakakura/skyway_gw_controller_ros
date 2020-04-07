#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType

from common import post


class PeerId:
    def __init__(self, peer_id):
        # type: (str) -> PeerId
        self.__peer_id = peer_id

    def id(self):
        # type: () -> str
        return self.__peer_id


# This method call POST /peer API to create PeerObject
# http://35.200.46.204/#/1.peers/peer
def create_peer(url, key, domain, peer_id, turn):
    # type: (str, str, str, PeerId, bool) -> ApiResponse
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
