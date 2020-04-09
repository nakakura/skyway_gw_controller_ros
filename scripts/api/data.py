#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType
import simplejson as json
from common import post, get, put, delete, ApiResponse
from peer import PeerInfo


class DataId:
    def __init__(self, data_id):
        # type: (str) -> DataId
        self.__data_id = data_id

    def id(self):
        # type: () -> str
        return self.__data_id


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
def create_data(url):
    # type: (str) -> ApiResponse
    return get("{}/data".format(url), 200)


# This method call DELETE /data API to delete a DataSocket
# http://35.200.46.204/#/2.data/data
def delete_data(url, data_id):
    # type: (str, DataId) -> ApiResponse
    return delete("{}/data/{}".format(url, data_id.id()), 204)


# This method call POST /data/connections API to establish P2P datalink
# http://35.200.46.204/#/2.data/data_connections_create
def connect(url, options):
    # type: (str, DataConnectionOptions) -> ApiResponse
    return post("{}/data/connections".format(url), options.json(), 202)
