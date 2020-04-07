#!/usr/bin/env python
# license removed for brevity
import os
import rospy

from api.peer import PeerInfo, create_peer
import const


def skyway():
    if not "API_KEY" in os.environ:
        raise rospy.ROSException, "Set API_KEY in environments"

    const.API_KEY = os.environ["API_KEY"]
    rospy.init_node("talker", anonymous=True)
    resp = create_peer(
        "http://localhost:8000",
        const.API_KEY,
        "localhost",
        PeerInfo("peer_id", ""),
        True,
    )
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rate.sleep()


if __name__ == "__main__":
    try:
        skyway()
    except rospy.ROSInterruptException:
        pass
