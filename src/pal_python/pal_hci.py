#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_hci.py
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

# TODO: Rename to pal_sound?

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

TTS_ACTION_NAME = '/sound'

global __tts_client
__tts_client = None

# TODO: Add a non-blocking version?
#       Possible design:
#         * sync:   TtsSpeak(text).wait()
#         * async:  s = TtsSpeak(text, opt_cb).start()
#                   s.status == True
def tts_speak(text, wait_before_speaking=0):
    """
    Lets the robot say the given text out aloud. This is a blocking call.
    """

    from text_to_speech.msg import SoundAction, SoundGoal

    global __tts_client
    if __tts_client is None:
        __tts_client = actionlib.SimpleActionClient(TTS_ACTION_NAME, SoundAction)
        rospy.logdebug('Waiting for "%s"...' % TTS_ACTION_NAME)
        if not __tts_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn('Couldn\'t connect to "%s" server.' % TTS_ACTION_NAME)
            return False

    tts_goal = SoundGoal()
    tts_goal.wait_before_speaking = rospy.Duration(wait_before_speaking)
    tts_goal.text = text

    rospy.logdebug('Sent speak command with "%s" (wait: %.3f seconds).' % (text,
        wait_before_speaking))
    __tts_client.send_goal(tts_goal)
    __tts_client.wait_for_result()

    result = __tts_client.get_state()
    rospy.logdebug('Result for last speech command: %s' % result)
    return result == GoalStatus.SUCCEEDED
