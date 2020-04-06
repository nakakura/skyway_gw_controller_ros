#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType


class PeerId:
    def __init__(self):
        self.name = "peer_id"


def create_peer(url, key, domain, peer_id, turn):
    if (
        isinstance(url, str)
        and isinstance(key, str)
        and isinstance(domain, str)
        and isinstance(peer_id, PeerId)
        and isinstance(turn, bool)
    ):
        rospy.logerr("isinstance true")

    headers = {"content-type": "application/json"}
    payload = {"key": key, "domain": domain, "peer_id": peer_id, "turn": turn}
    resp = requests.post(
        url + "/peers", headers=headers, json=payload, timeout=(3.0, 1.0)
    )
    rospy.logerr("hoge")
    rospy.logerr(resp.json())

    if resp.status_code == 201:
        print(json.dumps(resp.json(), indent=2))
    elif resp.status_code == 202:
        print("hoge")
    else:
        print("hoge")

    return resp.json()
