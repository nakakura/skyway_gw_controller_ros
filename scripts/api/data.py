#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType
import simplejson as json
import multiprocessing
import Queue
import sys
import const

from rest import post, get, put, delete, ApiResponse
from peer import PeerInfo


class DataId:
    def __init__(self, data_id):
        # type: (str) -> DataId
        self.__data_id = data_id

    def id(self):
        # type: () -> str
        return self.__data_id


class DataConnectionId:
    def __init__(self, data_connection_id):
        # type: (str) -> DataConnectionId
        self.__data_connection_id = data_connection_id

    def id(self):
        # type: () -> str
        return self.__data_connection_id


class DataConnectionOptions:
    def __init__(self, peer_info, options, target_id, data_id, redirect_params):
        # type: (PeerInfo, dict, str, DataId, dict) -> DataConnectionOptions
        self.__peer_id = peer_info.id()
        self.__token = peer_info.token()
        self.__options = options
        self.__target_id = target_id
        self.__params = {"data_id": data_id.id()}
        self.__redirect_params = redirect_params

    def json(self):
        # type: () -> dict
        return {
            "peer_id": self.__peer_id,
            "token": self.__token,
            "options": self.__options,
            "target_id": self.__target_id,
            "params": self.__params,
            "redirect_params": self.__redirect_params,
        }


# This method call GET /data API to open a DataSocket
# http://35.200.46.204/#/2.data/data
def create_data():
    # type: () -> ApiResponse
    return get("{}/data".format(const.URL), 200)


# This method call DELETE /data API to delete a DataSocket
# http://35.200.46.204/#/2.data/data
def delete_data(data_id):
    # type: (DataId) -> ApiResponse
    return delete("{}/data/{}".format(const.URL, data_id.id()), 204)


# This method call POST /data/connections API to establish P2P datalink
# http://35.200.46.204/#/2.data/data_connections_create
def connect(options):
    # type: (DataConnectionOptions) -> ApiResponse
    return post("{}/data/connections".format(const.URL), options.json(), 202)


# This method call DELETE /data/connections API to close P2P datalink
# http://35.200.46.204/#/2.data/data_connection_close
def disconnect(data_connection_id):
    # type: (DataConnectionId) -> ApiResponse
    return delete(
        "{}/data/connections/{}".format(const.URL, data_connection_id.id()), 204
    )


# This method call PUT /data/connections API to redirect data received from datalink
# http://35.200.46.204/#/2.data/data_connection_put
def redirect(data_id, data_connection_id, redirect):
    # type: (DataId, DataConnectionId, dict) -> ApiResponse
    body = {"feed_params": {data_id: data_id.id()}, "redirect_params": redirect}
    return put(
        "{}/data/connections/{}".format(const.URL, data_connection_id.id()), body, 200
    )


# This method call GET /data/connections API to get events from datalink
# http://35.200.46.204/#/2.data/data_connection_events
def events(data_connection_id):
    # type: (DataConnectionId) -> ApiResponse
    return get(
        "{}/data/connections/{}/events".format(const.URL, data_connection_id.id()), 200
    )


# This method call GET /data/connections API to get events from datalink
# http://35.200.46.204/#/2.data/data_connection_events
def status(data_connection_id):
    # type: (DataConnectionId) -> ApiResponse
    return get(
        "{}/data/connections/{}/status".format(const.URL, data_connection_id.id()), 200
    )


# When a DataChannel is established by a neighybour,
# this function decline or redirect data from neighbour to an user program
# according to the configuration
def on_connect(config, data_connection_id, key):
    # type: (list, DataConnectionId, str) -> None
    for parameter in config:
        if parameter["name"] == key:
            # Anticipated connection
            # redirect as configuration
            resp = create_data().json()
            data_id = DataId(resp["data_id"])
            if "redirect_params" in parameter:
                redirect_params = parameter["redirect_params"]
            else:
                redirect_params = {}
            redirect(data_id, data_connection_id, redirect_params)
            # FIXME notify connection information to user
            return
    # not intended connection
    # close it
    _resp = disconnect(data_connection_id).json()


# event driven process
# events are basically from GET /peer/events
# and user programs via ROS channel.
# only APP_CLOSING is triggerd by ROS itself
def on_events(config, queue):
    # type: (list, multiprocessing.Queue) -> None
    # polling while ros is running
    while True:
        try:
            message = queue.get(timeout=0.2)
            if message["type"] == "CONNECTION":
                # ROS sends SIGTERM
                data_connection_id = DataConnectionId(message["data_connection_id"])
                resp = status(data_connection_id).json()
                on_connect(config, data_connection_id, resp["metadata"])
                # FIXME this function should store DataConnectionIds to handle DataConnections
                continue
            elif message["type"] == "APP_CLOSING":
                # ROS sends SIGTERM
                # FIXME this function should close DataChannels in this timing
                break
        except Queue.Empty as e:
            continue
        except Exception as e:
            rospy.logerr(sys.exc_info())
            rospy.logerr("We lacked patience and got a multiprocessing.TimeoutError")
