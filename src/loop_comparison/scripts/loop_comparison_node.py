#!/usr/bin/env python3
import sys
from loop_comparison.loop_runner import main

if __name__ == '__main__':
    main(sys.argv[1:], loop_comparison=True)
