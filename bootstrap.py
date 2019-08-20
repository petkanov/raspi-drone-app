from subprocess import Popen, PIPE
import os, signal
import time

p = Popen('python3 sender.py --videokey=KH46OP-sD7', shell=True)
p2 = Popen(['python3', 'receiver.py'])

time.sleep(38)


try:
    """
    PROCESS_TERMINATE = 1
    handle = win32api.OpenProcess(PROCESS_TERMINATE, False, p.pid)
    win32api.TerminateProcess(handle, -1)
    win32api.CloseHandle(handle)
    """
    os.killpg(os.getpgid(p.pid), 15)  #LInux version
    """
    os.kill(p.pid, signal.SIGINT)
    os.kill(p.pid, signal.CTRL_C_EVENT)
    os.kill(p.pid, signal.CTRL_BREAK_EVENT) """
except Exception as e:
    print('woopsy'+str(e))
p.kill()
p.terminate()




try:
    os.killpg(os.getpgid(p2.pid), 15)  #LInux version
except Exception as e:
    print('woopsy'+str(e))
    
p2.kill()
p2.terminate()
