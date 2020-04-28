#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from scripts.api import data
from scripts.api.data import DataConnectionId
import multiprocessing

from scripts.error import MyError

# load config for the new DataConnection
# if the new DataConnection is invalid, raise Error
def load_data_connection_config(config, data_connection_id):
    # type: (list, DataConnectionId) -> dict

    # check status of new DataConnection
    result = data.status(data_connection_id).json()
    # check metadata of the status
    metadata = result["metadata"]
    for param in config:
        if "name" in param and metadata == param["name"]:
            # return its ID and configuration, if the connection is valid
            return param
    # Disconnect and raise error if the connection is invalid
    _result = data.disconnect(data_connection_id)
    raise MyError("Invalid DataConnection. It was closed.")


# load config for the new DataConnection
# if the new DataConnection is invalid, raise Error
def create_redirect_params(config):
    # type: (dict) -> (dict, dict)

    # open Data Socket for response
    result = data.create_data().json()
    # create redirect parameter to send the data to End-User-Program
    redirect_params = {}
    if "redirect_params" in config:
        redirect_params = config["redirect_params"]
    # parameter for PUT /data/connections/{data_connection_id}
    json = {
        "feed_params": {"data_id": result["data_id"]},
        "redirect_params": redirect_params,
    }
    return (json, result)


def on_connect(queue, config, data_connection_id):
    # type: (multiprocessing.Queue, list, DataConnectionId) -> list
    try:
        params = load_data_connection_config(config, data_connection_id)
    except MyError:
        # just close invalid connection
        return config
    else:
        (redirect_params, data_sock_info) = create_redirect_params(params)
        result = data.redirect(
            data_sock_info["data_id"], data_connection_id, redirect_params
        )
        if not result.is_ok():
            raise MyError(
                "WebRTC Gateway returns invalid data"
                "as a response of PUT /data/connections/DATA_CONNECTION_ID:"
                "{}".format(result.err())
            )

        queue.put(
            {
                "type": "CONNECTION",
                "value": {"redirect_params": redirect_params, "data": result.json()},
            }
        )
        # redirect success
        # remove redirect information which is used for this connection
        return [x for x in config if x["name"] != params["name"]]
