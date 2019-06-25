import time
from selfdrive.df import lib_main
ffi, libmpc = lib_main.get_libmpc()
libmpc.init_model()
start = time.time()
for i in range(100):
  model_output = libmpc.run_model(0.8653078153514447,
   0.46805728618371273,
   0.46805728618371273,
   0.28780443294609244,
   0.01075646532123655)
print(time.time() - start)
print(model_output)
