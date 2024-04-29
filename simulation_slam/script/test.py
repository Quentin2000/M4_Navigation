
import numpy as np

input = [100, 100, 100, 0, 0, 0, 0, 0, 0, 0,
   100, 100, 100, 0, 0, 0, 0, 0, 0, 0,
   100, 100, 100, 0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0,
   0,   0,   0,   0, 0, 0, 0, 0, 0, 0]

input_np = np.array(input)

output = np.minimum(1.0*input_np + 0.5*input_np, 100).astype(np.int8)

print(output)

output_list = output.tolist()

print(output_list)

print(type(output_list))