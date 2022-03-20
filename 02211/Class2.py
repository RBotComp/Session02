# # name = "My name is Mehul. \n" it is ????????????
# # print (name - "ul")


# # number = 1
# # numbers = [1, 2, 3, 4, 5, 6]
# # #Arrays:   0  1  2  3  4  5
# # print(numbers[0])
# import random

# file = open("02211/numbers.txt", "w")
# # file.write("hi\n")
# file.write(str(random.randint(0,100)))
# # for line in file:
# #     if (line[:-1].islower()):
# #         print(line)
















































#Lambda --> Greek Letter. 

'''
Ultrasonic Sensor: getDist(), getDistCM(), getDistBackwards()

function: usDistance == csColor()

'''

def isEqual(a, b):
    return a==b

def isEqual2(func, x, y):

    return func(x/2,y)

print(isEqual2(lambda x, y: isEqual(x, y), 10, 5))