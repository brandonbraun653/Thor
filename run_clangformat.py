# **********************************************************************************
#   FileName:
#       run_clangformat.py
#
#   Description:
#       Runs custom clang-formatting options on the Thor project
#
#   Usage Examples:
#       python run_clangformat.py
#
#   2019 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************

import os
import sys

import dev.tools.swqa as swqa
from dev.tools.swqa.clangformat import ClangFormatter


this_dir = os.path.join(os.path.dirname(__file__))
print("This is the working dir: {}".format(this_dir))

format_file = os.path.join(this_dir, 'thor_clangformat.json')
clang_format = os.path.join(swqa.swqa_dir, "bin\\clang-format.exe")
working_dir = this_dir


def run_clang_format():
    """
    Executes clang formatting on the Thor project

    :return: Process status code
    :rtype: int
    """
    cf = ClangFormatter(project_file=format_file, working_dir=working_dir, clang_format_exe=clang_format)
    return cf.execute()


if __name__ == "__main__":
    sys.exit(run_clang_format())

