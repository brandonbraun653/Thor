# **********************************************************************************
#   FileName:
#       run_cppcheck.py
#
#   Description:
#       Executes cppcheck on the Thor project files
#
#   Usage Examples:
#       N/A
#
#   2019 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************

import os
import sys

from dev.tools.swqa.cppcheck import CppCheck

this_dir = os.path.dirname(__file__)
cppcheck_path = os.path.expandvars('$GITHUB_ROOT/cppcheck/bin/cppcheck.exe')
config_file = os.path.join(this_dir, 'thor_cppcheck.json')


def run_cpp_check():
    """
    Runs CppCheck on the Thor project

    :return: Process status code
    :rtype: int
    """
    checker = CppCheck(exe_path=cppcheck_path)
    checker.load_config(file=config_file)
    return checker.execute(config='stm32f767_hal', working_dir=this_dir)


if __name__ == "__main__":
    sys.exit(run_cpp_check())
