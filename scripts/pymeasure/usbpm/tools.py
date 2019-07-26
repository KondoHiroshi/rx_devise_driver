from serial.tools import list_ports

from . import ma24126a

anritsu_vendor_id = 0x0b5b
pm_name_dic = {
        'ma24126a': 0xffcf
        }

def open(pm_name):
    pm_list = list_ports.comports()
    
    for pm in pm_list:
        if pm_name == 'ma24126a':
            if pm.pid == pm_name_dic['ma24126a']:
                p = ma24126a.ma24126a_driver(pm.device)
                return p
            else: pass

        continue
