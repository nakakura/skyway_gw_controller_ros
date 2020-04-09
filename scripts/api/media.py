#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType
import simplejson as json
from common import post, get, put, delete, ApiResponse


class MediaId:
    def __init__(self, media_id):
        # type: (str) -> DataId
        self.__media_id = media_id

    def id(self):
        # type: () -> str
        return self.__media_id


# This method call POST /media API to open a MediaSocket
# http://35.200.46.204/#/3.media/media
def create_media(url, is_video):
    # type: (str, bool) -> ApiResponse
    return post("{}/media".format(url), {"is_video": is_video}, 201)


# This method call DELETE /media/{media_id} API to close a MediaSocket
# http://35.200.46.204/#/3.media/streams_delete
def delete_media(url, media_id):
    # type: (str, MediaId) -> ApiResponse
    return delete("{}/media/{}".format(url, media_id.id()), 204)


# This method call POST /media/rtcp API to open a RtcpSocket
# http://35.200.46.204/#/3.media/media_rtcp_create
def create_rtcp(url):
    # type: (str) -> ApiResponse
    return post("{}/media/rtcp".format(url), {}, 201)
