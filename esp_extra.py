from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

env.Replace(
    MYUPLOADERFLAGS=[
        "-vv",
        "-cd", "$UPLOAD_RESETMETHOD",
        "-cb", "$UPLOAD_SPEED",
        "-cp", "$UPLOAD_PORT",
        "-ca", "0x00000",
        "-cf", "$SOURCE"
    ],
    UPLOADCMD='$UPLOADER $MYUPLOADERFLAGS',
)
