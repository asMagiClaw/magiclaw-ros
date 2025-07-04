# !/usr/bin/env python3

"""
MagicLaw ZMQ Module
====================

This module provides ZeroMQ-based communication for the MagicLaw system, including publishers and subscribers
for various components such as camera, claw, finger, phone, and MagiClaw.
"""

from .magiclaw import MagiClawPublisher, MagiClawSubscriber