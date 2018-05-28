#!/usr/bin/env python

from distutils.core import setup

setup(name='Pool Controller',
      version='0.1.0',
      description='Monitor for Adafruit Feather Pool Controller',
      author='Stuart B. Wilkins',
      author_email='stuwilkins@mac.com',
      packages=['PoolController'],
      scripts=['scripts/pclogger']
      )
