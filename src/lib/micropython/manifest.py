# manifest.pyinclude("$(MPY_DIR)/extmod/uasyncio")
# include("modules/testmod/manifest.py")
module("testmod.py", base_path="modules/testmod/")
require("base64")
require("collections")
# require("datetime")
require("time")
require("cbor2")
