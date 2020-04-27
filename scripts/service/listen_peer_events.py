# -*- coding: utf-8 -*-
import os
import rospy
import sys
from concurrent import futures
import time
import multiprocessing
import Queue
import random
import const

from scripts.api import peer

# FIXME
class MyError(Exception):
    pass


def listen_peer_events(peer_info, control_queue, media_event_queue, data_event_queue):
    # type: (PeerInfo, Queue, Queue, Queue) -> None
    # polling while ros is running
    while True:
        try:
            message = control_queue.get(timeout=0.1)
            if message["type"] == "APP_CLOSING":
                break
        except Queue.Empty as e:
            pass
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")
            raise e

        # get an event
        resp = peer.listen_event(peer_info)
        if not resp.is_ok():
            err = resp.err()
            # polling timeout
            if "status" in err and err["status"] == 408:
                continue
            else:
                raise MyError(err)

        json = resp.json()
        if not "event" in json:
            continue
        elif json["event"] == "CLOSE":
            break
        elif json["event"] == "OPEN":
            params = json["params"]
            peer_info = peer.PeerInfo(params["peer_id"], params["token"])
        elif json["event"] == "CONNECTION":
            params = json["data_params"]
            params["type"] = "CONNECTION"
            data_event_queue.put(params)
