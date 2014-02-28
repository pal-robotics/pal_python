#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_common.py
#
# Copyright (c) 2013 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
# 
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Siegfried-A. Gevatter

import xmlrpclib

import rospy
import rosgraph
from roslib import scriptutil

def is_simulation():
    """
    Return true if the node is running in simulation.
    """
    try:
        return rospy.get_param('use_sim_time')
    except (rosgraph.masterapi.MasterError, KeyError):
        return False

def is_node_running(node):
    """
    Return true if the given node is running.
    """
    master = scriptutil.get_master()
    node_info = master.lookupNode(rospy.get_name(), node)
    return node_info[0] == 1

# TODO: Move this to another file?
# TODO: Replace return value with exceptions?
def stop_node(node):
    """
    Stop the given node. Return true on success.
    """
    master = scriptutil.get_master()
    node_info = master.lookupNode(rospy.get_name(), node)
    [code, msg, uri] = master.lookupNode(rospy.get_name(), node)
    if code != 1:
        if msg.startswith('unknown node'):
            rospy.loginfo('Unknown node "%s"; ignored stop request.', node)
            return True
        else:
            rospy.logwarn('Couldn\'t stop "%s", problem retrieving information: %s', node, str(e))
            return False
    try:
        proxy = xmlrpclib.ServerProxy(uri)
        result = proxy.shutdown(rospy.get_name(), 'pal_common')
    except Exception, e:
        rospy.logwarn('Couldn\'t stop "%s": %s', node, str(e))
        return False
    return result[0] == 1

# Decorator for defining class (vs. instance) properties:
class classproperty(property):
    def __get__(self, cls, owner):
        return classmethod(self.fget).__get__(None, owner)()
