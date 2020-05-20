Import("env")

# retrieve build flags
my_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = {k: v for (k, v) in my_flags.get("CPPDEFINES")}

version_string = defines.get("VERSION_STRING")  # e.g. "1.2.2"
board_name = env["BOARD"]  # e.g. "esp01m"

# replace dots in version if linker can't find the path
#version_string = version_string.replace(".","_")

# set board and version in firmware name
env.Replace(PROGNAME="mavesp-{}-{}".format(board_name, version_string))