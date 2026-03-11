#!/bin/sh

cd lib/gscsp

python2 waf distclean

python2 waf configure

python2 waf build