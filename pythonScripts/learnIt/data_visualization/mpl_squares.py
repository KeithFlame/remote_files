# -*- coding: utf-8 -*-
# @Author: Keith W.
# @Date:   05.06.2022

import matplotlib.pyplot as plt
# import matplotlib.backends._backend_tk as tkagg

input_value = [1, 2, 3, 4, 5]
squares = [1, 4, 9, 16, 25]

plt.plot(input_value, squares, linewidth = 5)

plt.title("Square Numbers", fontsize = 24)
plt.xlabel("Value", fontsize = 14)
plt.ylabel("Square of Value", fontsize = 14)

plt.tick_params(axis = 'both', labelsize = 14)

plt.show()

