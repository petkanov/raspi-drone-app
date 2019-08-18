import time
import math

def decoratorBasic(func_obj):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func_obj(*args, **kwargs)
        end = time.time()
        print("Time took: ", func_obj.__name__, end-start)
        return result
    return wrapper


def decoratorWithParams(*args, **kwargs): 
    print("Inside decorator") 
    def wrapper(func_obj): 
        print("Inside Wrapper, decorator params: ", kwargs['param1']) 
        return func_obj
    return wrapper 
  

@decoratorBasic
def factorial(number):
    time.sleep(2)
    return math.factorial(number)
    
@decoratorWithParams(param1 = "First Parameter") 
def function(): 
     print( "Inside actual function")

  
print( factorial(100) )

function()