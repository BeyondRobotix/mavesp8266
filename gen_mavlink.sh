#!/bin/sh

mkdir -p generated
mavgen.py --lang C -o generated/mavlink --wire-protocol=1.0 mavlink/message_definitions/v1.0/ardupilotmega.xml
