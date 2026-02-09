#!/bin/bash
set -e
# Execute command with login shell (for conda initialization)
# Use +m flag to disable job control and suppress warnings
if [ $# -eq 0 ]; then
    # No arguments, just start login shell
    exec bash -l +m
elif [ $# -eq 1 ] && [ "$1" = "bash" ]; then
    # Single "bash" argument - start interactive shell
    exec bash -l +m
else
    # Execute command with login shell
    # Use "$*" to join all arguments into a single command string
    exec bash -l +m -c "$*"
fi

