#!/bin/bash

gunicorn --bind 0.0.0.0 --worker-class eventlet -w 1 kmp8servo:app

