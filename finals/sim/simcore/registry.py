"""The world singleton + command queue + sim thread. ALL PyBullet calls happen here (PyBullet is not thread-safe). Phase 1.

Simulator INTERNAL — mission code must never import simcore.
"""
