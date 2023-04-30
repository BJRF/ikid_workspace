# import numpy as np
# import matplotlib.pyplot as plt


# # 读取文本文件中的数据
# with open("/home/hjf/project/ikid_workspace/result_data/distance_result/distance_result_50_200", "r") as f:
#     data = f.readline().strip().split(" ")
#     data = [float(x) for x in data]


# # 将数据转换为NumPy数组
# data_array = np.array(data)


# # 创建x轴数据（使用数组下标）
# x = np.arange(len(data_array))


# # 绘制图表
# plt.plot(x, data_array)
# plt.xlabel("Index")
# plt.ylabel("Value")
# plt.title("Data Plot")
# plt.show()



import numpy as np
import matplotlib.pyplot as plt

file_name = "distance_result_50_200"
kf_file_name = "kf_distance_result_50_200"


# 读取第一个文本文件中的数据
with open("/home/hjf/project/ikid_workspace/result_data/distance_result/" + file_name, "r") as f:
    data1 = f.readline().strip().split(" ")
    data1 = [float(x) for x in data1]


# 读取第二个文本文件中的数据
with open("/home/hjf/project/ikid_workspace/result_data/distance_result/" + kf_file_name, "r") as f:
    data2 = f.readline().strip().split(" ")
    data2 = [float(x) for x in data2]


# 将数据转换为NumPy数组
data_array1 = np.array(data1)
data_array2 = np.array(data2)


# 创建x轴数据（使用数组下标）
x = np.arange(len(data_array1))


# 绘制图表
plt.plot(x, data_array1, label=file_name)
plt.plot(x, data_array2, label=kf_file_name)
plt.xlabel("Index")
plt.ylabel("Distance")
plt.title(file_name)
plt.legend()
plt.show()

