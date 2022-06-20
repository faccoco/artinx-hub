import shutil
import os
import sys


def copy_directory(src, dest):
    """
    :param src: the source directory. must not be a single file.
    :param dest: the source directory. must not be a single file.
    :return: void
    """
    assert (os.path.isdir(src) and os.path.isdir(dest))
    src_files = os.listdir(src)
    for file_name in src_files:
        full_file_name = os.path.join(src, file_name)
        if os.path.isfile(full_file_name):
            shutil.copy(full_file_name, dest)


try:
    parent_directory = os.path.dirname(os.path.abspath(__file__)) + "/.."
    print(parent_directory)
    os.chdir(parent_directory)
    copy_directory("config_templates/", "config/")
except IOError as e:
    print("Unable to copy file. %s" % e)
except:
    print("Unexpected error:", sys.exc_info())
