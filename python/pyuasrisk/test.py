import os
import sys
import subprocess

# Build the python module
curdir = os.path.abspath(os.path.dirname(__file__))
subprocess.check_call([sys.executable, os.path.join(curdir, 'setup.py'), 'build_ext', '--inplace'], cwd=curdir)

