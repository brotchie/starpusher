#!/bin/bash

watchexec \
    -r \
    -w . \
    -w ../main \
    "make viz && ./viz"
