import datetime
import subprocess

Import("env")

# get revivion from Git
revision = (
    subprocess.check_output(["git", "rev-parse", "--short", "HEAD"])
    .strip()
    .decode("utf-8")
)
#get current date and time
curr_date = datetime.datetime.now()
date_str = f"{curr_date.year}-{curr_date.month:02}-{curr_date.day:02}"
time_str = f"{curr_date.hour:02}:{curr_date.minute:02}:{curr_date.second:02}"

# add revision and date/time as build informations to the existing build flags (workaround to fix compilation issue with "date +%%..." )
build_flags = env['BUILD_FLAGS']
build_flags[0] = "!echo" + ' "-DPIO_SRC_REV="' +  str(revision)  + ' "-DPIO_BUILD_DATE="' + date_str + ' "-DPIO_BUILD_TIME="' +  time_str + " " + build_flags[0]

# retrieve build flags
my_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}
version_string = defines.get("VERSION_STRING")  # e.g. "1.2.2"
board_name = env["BOARD"]  # e.g. "esp01m"

# replace dots in version if linker can't find the path
#version_string = version_string.replace(".","_")

# set board and version in firmware name
env.Replace(PROGNAME="mavesp-{}-{}".format(board_name, version_string))