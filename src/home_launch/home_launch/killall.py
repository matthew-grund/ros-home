#!/usr/bin/python3

import subprocess


def ros_home_processes():
    rhp=[]
    cmd = ['ps','axo','uname,pid,cmd']
    ret = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    psstr=ret.stdout.decode('utf-8')
    pslines=psstr.splitlines()
    pslines=pslines[1:]
    for line in pslines:
        #print(line)
        p={}
        field=line.split()
        #print(field)
        p['uname']=field[0]
        p['pid']=field[1]
        cmd=field[2:]
        p['cmd']=cmd
        cmdstr=" ".join(cmd)
        if 'home_core' in cmdstr:
            rhp.append(p)
        elif 'home_devices' in cmdstr:
            rhp.append(p)
        elif 'home_extras' in cmdstr:
            rhp.append(p)
    return rhp

def kill_pid(pid):
    cmd=['kill',pid]
    ret = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    return

def main():
    rhp = ros_home_processes()
    # print(rhp)
    for p in rhp:
        kill_pid(p['pid'])
    return


# call the main if we are called directly
if __name__ == '__main__':
    main()


