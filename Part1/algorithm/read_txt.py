import ast

f = open('map1.txt')
routs = []
temp = []
for line in f:
    # print(line)
    temp.append(line)
#
for i in range(len(temp)):
    routs.append(ast.literal_eval(temp[i]))

print(routs[0][0])