#!/usr/bin/env python
# license removed for brevity
import os
import rospy

from api.peer import PeerInfo, create_peer, delete_peer
import const


def skyway():
    if not "API_KEY" in os.environ:
        raise rospy.ROSException, "Set API_KEY in environments"

    const.API_KEY = os.environ["API_KEY"]
    const.URL = "http://localhost:8000"

    rospy.init_node("talker", anonymous=True)
    resp = create_peer(
        const.URL, const.API_KEY, "localhost", PeerInfo("peer_id", ""), True,
    ).json()
    peer_info = PeerInfo(resp["params"]["peer_id"], resp["params"]["token"])

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rate.sleep()

    resp = delete_peer(const.URL, peer_info)
    rospy.logerr(resp.json())


if __name__ == "__main__":
    try:
        skyway()
    except rospy.ROSInterruptException:
        pass
