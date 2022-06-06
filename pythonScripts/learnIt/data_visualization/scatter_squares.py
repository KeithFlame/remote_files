# -*- coding: utf-8 -*-
# @Author: Keith W.
# @Date:   05.06.2022

import matplotlib.pyplot as plt

x = list(range(1,1001))
y = [xt**2 for xt in x]
# y = [1, 4, 9, 16, 25]
plt.scatter(x, y, c = 'red', edgecolor = 'none', s = 40)
plt.scatter(x, y, c = y, cmap = plt.cm.Blues, edgecolor = 'none', s = 40)

plt.title("Square Number", fontsize = 24)
plt.xlabel("Value", fontsize = 14)
plt.ylabel("Square of Number", fontsize = 14)

plt.tick_params(axis = 'both', which = 'major', labelsize = 14)
plt.axis([0, 1100, 0, 1100000])
plt.show()

plt.savefig('./pictures/square_plot.tiff', bbox_inches = 'tight')