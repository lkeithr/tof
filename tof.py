import os
from Espros_API import *
from GCC_API import *
import argparse
import cmd
import subprocess
import threading
import sys
import time

def run_espros():
    print('Running Espros')
    Espros_Commands().cmdloop()

def run_gcc():
    print('Running GCC')
    GCC_Commands().cmdloop()

def run_gcc_old():
    print('Running GCC old')
    GCC_Commands_Old().cmdloop()
    
def run_gcc_gui():
    print('Launching ToF GUI')
    event = threading.Event() # need this to ensure thread stops correctly
    t2 = threading.Thread(target=flush_stdout, args=(event, ))
    t2.daemon = True
    t2.start()
    GCC_Commands().cmdloop()
    # I don't understand why the following code doesn't work - it is supposed
    # to direct the output to sys.stdout which I thought would be what we want
    '''myCmd = cmd.Cmd()
    myCmd.use_rawinput = False
    GCC_Commands().cmdloop(myCmd)'''
    
def flush_stdout(event: threading.Event): # forces everything on CLI screen to
# go to stdout regardless of execution status (otherwise will wait until CLI method
# is complete then print to GUI)
    while True:
        sys.stdout.flush()
        time.sleep(0.5)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch a CLI to control the EPC660 ToF Camera")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-e", "--espros",
                        help="Open an interface with all Espros SDK commands",
                        action="store_true")
    group.add_argument("-g", "--gcc",
                        help="Open an interface with all GCC API commands",
                        action="store_true")
    group.add_argument("-g_old", "--gcc_old",
                       help="Open an interface with all GCC API commands (old version)",
                       action="store_true")
    group.add_argument("-gui", "--gui",
                       help="Launches a GUI to interface with all GCC API commands", 
                       action="store_true")
    group.add_argument("-i", "--stdin",
                       action="store")
    group.add_argument("-o", "--stdout",
                       action="store")
    args = parser.parse_args()

    if not args.espros and not args.gcc and not args.gcc_old and not args.gui:
        print("Improper usage, see help below")
        os.system('python3 tof.py -h')
    elif args.espros:
        run_espros()
    elif args.gcc:
        run_gcc()
    elif args.gcc_old:
        run_gcc_old()
    elif args.gui:
        run_gcc_gui()