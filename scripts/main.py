#!/usr/bin/env python
# license removed for brevity
import os
import rospy
import sys
from concurrent import futures
import time
import multiprocessing
import Queue
import random
import const

from api.peer import PeerInfo, create_peer, delete_peer, listen_event
from api import data

# This method is responsible for deleting peer object when rospy close
def runloop(queue, peer_info):
    # type: (Queue, PeerInfo) -> None
    rate = rospy.Rate(10)  # 10hz
    # wait for ros termination
    while True:
        try:
            rate.sleep()
            message = queue.get(timeout=0.2)
            if message["type"] == "APP_CLOSING":
                break
        except Queue.Empty as e:
            continue
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")

    # delete Peer Object
    resp = delete_peer(const.URL, peer_info)
    if not resp.is_ok():
        raise resp.err()


def listen_peer_events(control_queue, peer_info, media_event_queue, data_event_queue):
    # type: (Queue, PeerInfo, multiprocessing.Queue, multiprocessing.Queue) -> None
    # polling while ros is running
    while True:
        try:
            message = control_queue.get(timeout=0.1)
            if message["type"] == "APP_CLOSING":
                data_event_queue.put({"type": "APP_CLOSING"})
                media_event_queue.put({"type": "APP_CLOSING"})
                break
        except Queue.Empty as e:
            continue
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")

        # get an event
        resp = listen_event(const.URL, peer_info)
        if not resp.is_ok():
            err = resp.err()
            # polling timeout
            if "status" in err and err["status"] == 408:
                continue
            else:
                raise err

        json = resp.json()
        if not "event" in json:
            continue
        elif json["event"] == "CLOSE":
            break
        elif json["event"] == "OPEN":
            params = json["params"]
            peer_info = PeerInfo(params["peer_id"], params["token"])
        elif json["event"] == "CONNECTION":
            params = json["data_params"]
            params["type"] = "CONNECTION"
            data_event_queue.put(params)


def media(queue):
    # type: (multiprocessing.Queue) -> None

    while True:
        try:
            message = queue.get(timeout=0.1)
            if message["type"] == "APP_CLOSING":
                break
        except Queue.Empty as e:
            continue
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")

        try:
            message = queue.get(timeout=0.2)
        except Queue.Empty as e:
            continue
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")


def main():
    if not "API_KEY" in os.environ:
        raise rospy.ROSException, "Set API_KEY in environments"

    # api key of SkyWay
    const.API_KEY = os.environ["API_KEY"]
    # SkyWay WebRTC Gateway endpoint
    const.URL = "http://localhost:8000"

    rospy.init_node("talker", anonymous=True)

    # Create a Peer Object
    resp = create_peer(
        const.URL, const.API_KEY, "localhost", PeerInfo("peer_id", ""), True,
    ).json()
    peer_info = PeerInfo(resp["params"]["peer_id"], resp["params"]["token"])

    rate = rospy.Rate(10)  # 10hz
    with futures.ThreadPoolExecutor(max_workers=8) as executor:
        peer_control_queue = multiprocessing.Queue()
        loop_control_queue = multiprocessing.Queue()
        media_event_queue = multiprocessing.Queue()
        data_event_queue = multiprocessing.Queue()
        # delete peer object when rospy close
        closing_future = executor.submit(runloop, loop_control_queue, peer_info)
        # poll events from PeerObject
        peer_event_future = executor.submit(
            listen_peer_events,
            peer_control_queue,
            peer_info,
            media_event_queue,
            data_event_queue,
        )

        data_future = executor.submit(data.on_events, data_event_queue)
        media_future = executor.submit(media, media_event_queue)
        # keep checking rospy status
        while not rospy.is_shutdown():
            rate.sleep()
        # exit rospy
        peer_control_queue.put({"type": "APP_CLOSING"})
        loop_control_queue.put({"type": "APP_CLOSING"})
        closing_future.result()
        peer_event_future.result()
        data_future.result()
        media_future.result()

    # notify and wait just in case
    rospy.signal_shutdown("finish")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
