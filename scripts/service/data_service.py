#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import Queue
import sys

import data_redirect
from api import data

# This class virtually & roughly join multiple queues
# Just try to pop an event from 1st queue and return its value.
# Only when 1st queue return timeout error, try to pop 2nd queue.
# This is very rough code and don't work if 1st queue returns value frequently.
class JoinQueue:
    def __init__(self, queue1, queue2):
        # type: (Queue, Queue) -> None
        self.__queue1 = queue1
        self.__queue2 = queue2

    def __get_event(self, queue, timeout):
        # type: (Queue, float) -> (bool, dict)
        try:
            message = queue.get(timeout=timeout)
        except Queue.Empty:
            return False, {}
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")
            raise e
        else:
            return True, message

    def pop(self, timeout):
        # type: (float) -> dict
        (flag, event) = self.__get_event(self.__queue1, timeout)
        if flag:
            return event
        (flag, event) = self.__get_event(self.__queue2, timeout)
        if flag:
            return event
        return {"type": "TIMEOUT"}


def on_event(queue, config):
    # type: (JoinQueue, list) -> None
    while True:
        event = queue.pop(0.1)

        if event["type"] == "TIMEOUT":
            continue
        elif event["type"] == "CONNECTION":
            config = data_redirect.on_connect(config, data.DataConnectionId("dcid"))
