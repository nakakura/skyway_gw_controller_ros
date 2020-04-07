#!/usr/bin/env python
# license removed for brevity
import os
import rospy
from concurrent import futures
import time
from multiprocessing import Queue
import random

from api.peer import PeerInfo, create_peer, delete_peer, listen_event
import const

is_running = True


# This method is responsible for deleting peer object when rospy close
def runloop(peer_info):
    # type: (PeerInfo) -> None
    global is_running
    # wait for ros termination
    rate = rospy.Rate(10)  # 10hz
    while is_running:
        rate.sleep()
    # delete Peer Object
    resp = delete_peer(const.URL, peer_info)
    if not resp.is_ok():
        raise resp.err()


def listen_peer_events(peer_info):
    # type: (PeerInfo) -> None
    global is_running
    rate = rospy.Rate(10)  # 10hz
    # polling while ros is running
    while is_running:
        # get an event
        resp = listen_event(const.URL, peer_info)
        if not resp.is_ok():
            err = resp.err()
            # polling timeout
            if "status" in err and err["status"] == 408:
                rospy.logerr("continue")
                continue
            else:
                raise err

        json = resp.json()
        if "event" in json and json["event"] == "CLOSE":
            break


def skyway():
    if not "API_KEY" in os.environ:
        raise rospy.ROSException, "Set API_KEY in environments"

    # flag to check rospy is running
    global is_running
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
        # delete peer object when rospy close
        f1 = executor.submit(runloop, peer_info)
        # poll events from PeerObject
        f2 = executor.submit(listen_peer_events, peer_info)
        # keep checking rospy status
        while not rospy.is_shutdown():
            rate.sleep()
        # exit rospy
        is_running = False
        f1.result()
        f2.result()

    # notify and wait just in case
    rospy.signal_shutdown("finish")
    rospy.spin()


if __name__ == "__main__":
    try:
        skyway()
    except rospy.ROSInterruptException:
        pass
