#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
import rospy
from typing import NewType
import simplejson as json
from common import post, get, put, delete, ApiResponse

# This method call POST /media API to open a MediaSocket
# http://35.200.46.204/#/3.media/media
def create_media(url, is_video):
    # type: (str, bool) -> ApiResponse
    return post("{}/media".format(url), {"is_video": is_video}, 201)
