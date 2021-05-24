
#!/usr/bin/env python
### ^ mark python ros node as executable

# import rospy
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float32

#test out storing of lists
import json

def main():

    lst = [123,'apple',True,None,-8.7]
    with open("test.txt","w") as fp:
        json.dump(lst,fp)


if __name__ == "__main__":
    main()

# for ii in range(3):
#     print(ii)
#     print('This is an example file that I\'m using VIM to edit')
#     #this is a comment that I'm using to test that my file is saving:
