import ast
import matplotlib.pyplot as plt
import numpy as np

time_list_1 = []
quality_list_1 = []
time_list_2 = []
quality_list_2 = []
time_list_3 = []
quality_list_3 = []

f1 = open('source/time_list_1.txt')
temp = []
for line in f1:
    temp.append(line)
for i in range(len(temp)):
    time_list_1.append(ast.literal_eval(temp[i]))

f2 = open('source/time_list_2.txt')
temp = []
for line in f2:
    temp.append(line)
for i in range(len(temp)):
    time_list_2.append(ast.literal_eval(temp[i]))

f3 = open('source/time_list_3.txt')
temp = []
for line in f3:
    temp.append(line)
for i in range(len(temp)):
    time_list_3.append(ast.literal_eval(temp[i]))

q1 = open('source/quality_list_1.txt')
temp = []
for line in q1:
    temp.append(line)
for i in range(len(temp)):
    quality_list_1.append(ast.literal_eval(temp[i]))

q2 = open('source/quality_list_2.txt')
temp = []
for line in q2:
    temp.append(line)
for i in range(len(temp)):
    quality_list_2.append(ast.literal_eval(temp[i]))

q3 = open('source/quality_list_3.txt')
temp = []
for line in q3:
    temp.append(line)
for i in range(len(temp)):
    quality_list_3.append(ast.literal_eval(temp[i]))

sum_quality = 0
for i in range(len(quality_list_2)):
    sum_quality += quality_list_2[i]
print(sum_quality)


# question
# plt.figure('roadmap3')
# plt.xlabel('time')
# plt.ylabel('path quality')
# # plt.scatter(time_list_1, quality_list_1, c='r', s=20)
# # plt.scatter(time_list_2, quality_list_2, c='b', s=20)
# # plt.scatter(time_list_3, quality_list_3, c='y', s=20)
# plt.show()

# # compare time
# plt.figure('compare_time')
# plt.xlabel('sample num')
# plt.ylabel('time')
# plt.scatter(np.arange(len(time_list_1)), time_list_1, c='r', s=20, label='roadmap 1')
# plt.scatter(np.arange(len(time_list_2)), time_list_2, c='b', s=20, label='roadmap 2')
# plt.scatter(np.arange(len(time_list_3)), time_list_3, c='y', s=20, label='roadmap 3')
# plt.legend()
# plt.show()
#
# # compare quality
# plt.figure('compare_quality')
# plt.xlabel('sample num')
# plt.ylabel('quality')
# plt.scatter(np.arange(len(quality_list_1)), quality_list_1, c='r', s=20, label='roadmap 1')
# plt.scatter(np.arange(len(quality_list_2)), quality_list_2, c='b', s=20, label='roadmap 2')
# plt.scatter(np.arange(len(quality_list_3)), quality_list_3, c='y', s=20, label='roadmap 3')
# plt.legend()
# plt.show()

