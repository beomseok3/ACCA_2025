# import os
# import sys
import subprocess

def set_param(node_name,param_name,value):
    try:
        command =f"ros2 param set /{node_name} {param_name} {value}"
        # a=os.system(command)
        # print(a)
        var = subprocess.check_output(command,shell=True,text=True)
        print(var)
    except:
        raise ValueError

    #     # return True
    # except subprocess.CalledProcessError as e: 
    #     print(f"error_os_system {e}")
        # return False
# b =set_param("bs_cropbox_filter","detection_area","[0.,3.,-10.,0.]")

# 함수 잘 만들고 terminal log 문자열 succsess확인하든가 -->  state machine 적용하고 처리확인y