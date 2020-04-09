#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType

from common import post, get, put, delete, ApiResponse


class DataId:
    def __init__(self, data_id):
        # type: (str) -> DataId
        self.__data_id = data_id

    def id(self):
        # type: () -> str
        return self.__data_id


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
