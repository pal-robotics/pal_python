#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pal_python: pal_hci.py
#
# Copyright (c) 2013, 2014 PAL Robotics SL. All Rights Reserved
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
#   * Paul Mathieu

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

import pal_device_msgs.srv as PDMS
import pal_device_msgs.msg as PDM

from .pal_common import configurable
from .pal_rpc import AsyncServiceClient


@configurable
class Color(object):

    defaults = {
        'rgb': [0.0, 0.0, 0.0],
        'blinking': False,
    }

    def __init__(self, cfg={}):
        self._read_config(cfg)


class ReemLedClient:
    """
    A simple client to access REEM's ear LEDs. Right now you will need one
    client per color you want to use. Don't worry though, there will only
    be one service client for everybody (per call type).

    Intended use:

        blue = Color({'rgb': [0.0, 0.0, 1.0]})
        led_cl = ReemLedClient(blue)
        led_cl.fire(1.0)  # will set the color of the ears to blue for 1 sec

    """
    _flat_color_client = None
    _blink_color_client = None

    def __init__(self, color):
        self._color = color
        if ReemLedClient._flat_color_client is None:
            ReemLedClient._flat_color_client = AsyncServiceClient(
                'ledManager/TimedColourEffect',
                PDMS.TimedColourEffect,
                persistent=True
            )
            ReemLedClient._blink_color_client = AsyncServiceClient(
                'ledManager/TimedBlinkEffect',
                PDMS.TimedColourEffect,
                persistent=True
            )

    def fire(self, duration=1.0):
        if self.color.blinking:
            self._fire_blink(duration)
        else:
            self._fire_flat(duration)

    def _fire_flat(self, duration):
        effect = PDMS.TimedColourEffectRequest()
        effect.leds.ledMask = PDM.LedGroup.LEFT_EAR | PDM.LedGroup.RIGHT_EAR
        effect.effectDuration = rospy.Time(duration)
        effect.priority = 96
        effect.color.r = self._color.rgb[0]
        effect.color.g = self._color.rgb[1]
        effect.color.b = self._color.rgb[2]
        self._flat_color_client.call(effect)

    def _fire_blink(self, duration):
        effect = PDMS.TimedBlinkEffectRequest()
        effect.leds.ledMask = PDM.LedGroup.LEFT_EAR | PDM.LedGroup.RIGHT_EAR
        effect.effectDuration = rospy.Time(duration)
        effect.firstColorDuration = rospy.Time(0.2)
        effect.secondColorDuration = rospy.Time(0.2)
        effect.priority = 96
        effect.firstColor.r = self._color.rgb[0]
        effect.firstColor.g = self._color.rgb[1]
        effect.firstColor.b = self._color.rgb[2]
        # second color will be black by default
        self._blink_color_client.call(effect)


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

    rospy.logdebug('Sent speak command with "%s" (wait: %.3f seconds).' % (
        text, wait_before_speaking))
    __tts_client.send_goal(tts_goal)
    __tts_client.wait_for_result()

    result = __tts_client.get_state()
    rospy.logdebug('Result for last speech command: %s' % result)
    return result == GoalStatus.SUCCEEDED
