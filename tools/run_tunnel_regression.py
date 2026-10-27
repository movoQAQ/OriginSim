#!/usr/bin/env python3
"""Run/replay the repository-owned five-tunnel and sensor-dropout regression."""

import sys

from tunnel_regression.runner import main


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as error:
        print('ERROR: ' + str(error), file=sys.stderr)
        raise SystemExit(2)
