import os

test_url = "C:\\Users\\hjf\\Desktop\\football_datasets\\football\\test"
runs_url = "C:\\Users\\hjf\\Desktop\\yolov5-master\\runs\\detect\\exp23\\labels"

res = 0.0
num = 0

text_dir = os.listdir(test_url)
for test_f in text_dir:
    if(test_f[-4:] == ".txt"):
        test_file_temp = open(test_url + "\\" + test_f)  # 返回一个文件对象
        line = test_file_temp.readline()  # 调用文件的 readline()方法
        line_num = 0
        center_x = 0.0
        center_y = 0.0
        print(line)
        while line:
            line_num = line_num + 1
            dict_temp = line.split(' ')
            center_x += float(dict_temp[1])
            center_y += float(dict_temp[2])
            wight = dict_temp[3]
            height = dict_temp[4]
            line = test_file_temp.readline()
        center_x = center_x / line_num
        center_y = center_y / line_num

        runs_dir = os.listdir(runs_url)
        for runs_f in runs_dir:
            # 遍历找到名字相同的文件
            if (runs_f == test_f):
                runs_file_temp = open(runs_url + "\\" + runs_f)
                line2 = runs_file_temp.readline()
                center_x2 = 0.0
                center_y2 = 0.0
                line_num2 = 0
                while line2:
                    line_num2 = line_num2 + 1
                    dict_temp2 = line2.split(' ')
                    center_x2 += float(dict_temp2[1])
                    center_y2 += float(dict_temp2[2])
                    wight = dict_temp2[3]
                    height = dict_temp2[4]
                    line2 = runs_file_temp.readline()
                center_x2 = center_x2 / line_num2
                center_y2 = center_y2 / line_num2
                num = num + 1

                x_bias = 1 - (abs(center_x2 - center_x) / center_x)
                y_bias = 1 - (abs(center_y2 - center_y) / center_y)
                res += (x_bias + y_bias) / 2
                runs_file_temp.close()
                break
        test_file_temp.close()
res = res / num;
print("准确率为 : ")
print(res)