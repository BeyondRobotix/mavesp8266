#!/bin/sh

rm -rf generated
mkdir -p generated
mavgen.py --lang C -o generated/mavlink --wire-protocol=2.0 mavlink/message_definitions/v1.0/ardupilotmega.xml
