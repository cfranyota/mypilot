from cffi import FFI
import os
import subprocess
try:
    subprocess.check_call(["make", "-j4"], cwd="/data/openpilot/selfdrive/df")
except:
    pass

def get_libmpc():
    libmpc_fn = "/data/openpilot/selfdrive/df/dynamic_follow.so"

    ffi = FFI()
    ffi.cdef("""    
    float run_model(float v_ego, float a_ego, float v_lead, float x_lead, float a_lead);
    void init_model();
    """)

    return (ffi, ffi.dlopen(libmpc_fn))
