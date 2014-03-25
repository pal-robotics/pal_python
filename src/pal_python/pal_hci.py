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
import copy
import time
from actionlib_msgs.msg import GoalStatus

import pal_device_msgs.srv as PDMS
import pal_device_msgs.msg as PDM

from .pal_common import configurable
from .pal_rpc import AsyncServiceClient


@configurable
class LedColor(object):

    defaults = {
        'rgb': [0.0, 0.0, 0.0],
        'blinking': False,
    }

    def __init__(self, cfg={}):
        self._read_config(cfg)

    def __eq__(self, other):
        if not isinstance(other, LedColor):
            return False
        return self.rgb == other.rgb and self.blinking == other.blinking


class ReemLedClient(object):
    """
    A simple client to access REEM's ear LEDs. Right now you will need one
    client per color you want to use. Don't worry though, there will only
    be one service client for everybody (per call type).

    Intended use:

        blue = LedColor({'rgb': [0.0, 0.0, 1.0]})
        led_cl = ReemLedClient(blue)
        led_cl.fire(1.0)  # will set the color of the ears to blue for 1 sec

    """
    _flat_color_client = None
    _blink_color_client = None
    _cancel_effect_client = None

    _last_color = None
    _last_effect = None

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
                PDMS.TimedBlinkEffect,
                persistent=True
            )
            ReemLedClient._cancel_effect_client = AsyncServiceClient(
                'ledManager/CancelEffect',
                PDMS.CancelEffect,
                persistent=True
            )

    def fire(self, duration=1.0):
        color = copy.deepcopy(self._color)
        if color == self._last_color:
            self._last_effect.update_duration(duration)
        else:
            old_effect = self._last_effect
            self._last_effect = self._start_effect(color, duration)
            self._last_color = color
            if old_effect:
                old_effect.cancel()

    def _start_effect(self, color, duration):
        if self._color.blinking:
            return _BlinkLedController(color, duration)
        else:
            return _ColorLedController(color, duration)

class _LedController(object):

    __client = None  # instance of AsyncServiceClient
    __last_id = 0
    __active_effect_id = None
    __effects_map = {}

    def __init__(self, client):
        self.__client = client

    def cancel(self):
        if self.__active_effect_id is not None:
            self.__cancel_request(self.__active_effect_id)
            self.__active_effect_id = None
        else:
            # We may still be awaiting the response for the last request. In
            # that case, we invalidate it.
            self.__last_id += 1

    def _send_request(self, *args):
        previous_request_id = self.__active_effect_id
        self.__active_effect_id = None

        self.__last_id += 1
        self.__client.call(*args, cb=lambda x: self.__request_cb(self.__last_id, x))

        if previous_request_id is not None:
            # Remember that LedManager keeps around all requests until they expire
            # or are cancelled.
            self.__cancel_request(previous_request_id)

    def __request_cb(self, cid, response):
        if cid == self.__last_id:
            self.__active_effect_id = response.effectId
        else:
            # We are getting the response for a request that's already outdated
            self.__cancel_request(response.requestId)

    def __cancel_request(self, request_id):
        cancel_request = PDMS.CancelEffectRequest()
        cancel_request.effectId = request_id
        ReemLedClient._cancel_effect_client.call(cancel_request)

class _ColorLedController(_LedController):

    _request = None

    def __init__(self, color, duration):
        super(_ColorLedController, self).__init__(ReemLedClient._flat_color_client)

        # Create request object
        effect = PDMS.TimedColourEffectRequest()
        effect.leds.ledMask = PDM.LedGroup.LEFT_EAR | PDM.LedGroup.RIGHT_EAR
        effect.priority = 96
        effect.color.r = color.rgb[0]
        effect.color.g = color.rgb[1]
        effect.color.b = color.rgb[2]
        self._request = effect

        # Send first LED command
        self.update_duration(duration)

    def update_duration(self, duration):
        # We replace any previous command with the new one.
        self._request.effectDuration = rospy.Time(duration)
        self._send_request(self._request)

class _BlinkLedController(_LedController):

    _request = None

    _desired_timeout = None
    _current_timeout = None
    _timer = None

    _MIN_DURATION = 0.4  # min. animation time (with hardcoded 0.2 in __init__)
    _CMD_DURATION = 10.0

    def __init__(self, color, duration):
        super(_BlinkLedController, self).__init__(ReemLedClient._blink_color_client)

        # Create request object
        effect = PDMS.TimedBlinkEffectRequest()
        effect.leds.ledMask = PDM.LedGroup.LEFT_EAR | PDM.LedGroup.RIGHT_EAR
        effect.firstColorDuration = rospy.Duration.from_sec(0.2)
        effect.secondColorDuration = rospy.Duration.from_sec(0.2)
        effect.priority = 96
        effect.firstColor.r = color.rgb[0]
        effect.firstColor.g = color.rgb[1]
        effect.firstColor.b = color.rgb[2]
        # second color will be black by default
        self._request = effect

        # Send first LED command
        duration = max(duration, self._MIN_DURATION)
        self._desired_timeout = time.time() + duration
        self._send_command()

        # Start timer
        self._reset_timer()

    def __del__(self):
        if self._timer:
            self._timer.shutdown()

    def cancel(self):
        if self._timer:
            self._timer.shutdown()
        super(_BlinkLedController, self).cancel()

    def update_duration(self, duration):
        duration = max(duration, self._MIN_DURATION)
        self._desired_timeout = time.time() + duration
        if not self._timer:
            self._reset_timer()

    def _send_command(self):
        self._current_timeout = time.time() + self._CMD_DURATION
        self._request.effectDuration = rospy.Duration.from_sec(self._CMD_DURATION)
        self._send_request(self._request)

    def _reset_timer(self):
        if self._timer:
            self._timer.shutdown()
        self._timer = rospy.Timer(rospy.Duration.from_sec(self._MIN_DURATION),
                                  self._timeout, oneshot=False)

    def _timeout(self, event):
        """
        This method is called periodically every _MIN_DURATION seconds.
        """
        remaining_duration = self._desired_timeout - time.time()
        if remaining_duration <= 0:
            self.cancel()
            self._timer.shutdown()
            self._timer = None
        elif remaining_duration > (self._MIN_DURATION - 0.1):
            if self._current_timeout < self._desired_timeout - 0.1:
                self._send_command()

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
