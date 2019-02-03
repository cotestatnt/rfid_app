import os
cmd = 'sudo ./rfid_reader -r'
so = os.popen(cmd).read()
print(so) 
