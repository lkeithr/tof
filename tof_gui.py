import tkinter as tk
from tkinter import ttk
import subprocess
import sys
import os
import queue
import select
import threading
from tkinter import messagebox
import cv2 as cv

stdFontSize = 12

class ToFGUI:
    def __init__(self, cliProcess):
        self.cliProcess = cliProcess
        
        
        
        # Creating root
        self.root = tk.Tk()
        width = self.root.winfo_screenwidth()
        height = self.root.winfo_screenheight()
        self.root.geometry('%dx%d' % (width, height))  
        self.root.title('GCC ToF Camera Control')
        
        
        # Creating tabs
        self.create_tabs()
        
        
        # Creating an image frame
        #self.create_image_frame()
        
        # Testing webcam methods
        #self.cam = cv.VideoCapture(0)
        #self.stream_webcam
        
        
        
        # Add "CLI"
        self.create_path_labels()
        self.create_stream_mode_option()
        self.create_save_option()
        self.create_stream_button()
        self.create_CLI_output()
        self.create_CLI_input()
        self.update_vars()
        self.cliInput.focus()
        #self.create_enter_button()
        
        
        #self.startup_clear_buffer()
        
        # Creating list of methods for tab-autocomplete
        
        
        # Creating a thread to check the output from the CLI and update the GUI
        self.event = threading.Event() # need this to ensure thread stops correctly
        self.t = threading.Thread(target=self.read_CLI_output, args=(self.event, ))
        self.t.daemon = True
        self.t.start()
        
        
        self.root.protocol('WM_DELETE_WINDOW', self.closing_protocol)
        
        
    def startup_clear_buffer(self):
        for line in self.cliProcess.stdout:
            self.cliOutput.insert(tk.END, '>> ' + line)
            print('hi')
        
    
    def create_tabs(self):   
        self.notebook = ttk.Notebook(self.root)
        self.mainTab = ttk.Frame(self.notebook)
        self.settingsTab = ttk.Frame(self.notebook)
        self.notebook.add(self.mainTab, text='Camera Control')
        self.notebook.add(self.settingsTab, text='Camera Settings')
        self.notebook.pack(expand=1, fill='both')
        
        
    def create_image_frame(self):
        self.imageFrame = tk.Frame(self.mainTab, bg='gray', width=800, height = 400, relief='ridge')
        self.imageFrame.pack()
        
        self.imageLabel = tk.Label(self.imageFrame, width=32, height=24, relief='ridge')
        self.imageLabel.pack(fill='both')
        
    
    def create_CLI_input(self):
        self.cliInputText = tk.StringVar()
        self.cliInput = ttk.Entry(self.mainTab, textvariable=self.cliInputText,
                                   font=('Arial', stdFontSize), width=50)
        self.cliInput.bind('<Return>', self.enter_command)
        self.cliInput.pack()
        
    
    # This method should be unnecessary as we have return key input, but 
    # keeping here for now if we decide to implement in the future - NHS 01/22/2024
    '''def create_enter_button(self):
        boldStyle = ttk.Style()
        boldStyle.configure('Bold.TButton', font=('Arial', 12, 'bold'))
        self.enterButton = ttk.Button(self.mainTab, text='Enter Command',
                                       style='Bold.TButton', command=self.enter_command)
        self.enterButton.pack()'''
        
        
    def create_CLI_output(self):
        # Scrollbar to see history
        self.cliOutputScrollbar = tk.Scrollbar(self.root)
        self.cliOutputScrollbar.pack(side='top', anchor='ne')
        
        # Output from GCC_API
        self.cliOutput = tk.Listbox(self.mainTab, width=100, 
                                           font=('Arial',stdFontSize), 
                                           yscrollcommand=self.cliOutputScrollbar.set)
        self.cliOutput.pack()
        
        
    def create_stream_mode_option(self):
        streamModeButtonStyle = ttk.Style()
        streamModeButtonStyle.configure('streamModeButton.TRadiobutton', font=('Arial', stdFontSize, 'bold'))
        self.streamMode = tk.StringVar(value='a')
        self.amplitudeButton = ttk.Radiobutton(self.mainTab, text='Amplitude', 
                                               variable=self.streamMode, 
                                               value='a', 
                                               style='streamModeButton.TRadiobutton')
        self.distanceButton = ttk.Radiobutton(self.mainTab, text='Distance', 
                                              variable=self.streamMode, 
                                              value='d', 
                                              style='streamModeButton.TRadiobutton')
        self.amplitudeButton.pack()
        self.distanceButton.pack()
        
    
    def create_save_option(self):
        self.saveState = tk.IntVar()
        self.saveCheckbox = tk.Checkbutton(self.mainTab, text='Save Image Data?', 
                                           font=('Arial', stdFontSize), 
                                           variable=self.saveState)
        self.saveCheckbox.pack()
        
        
    def create_stream_button(self):
        boldStyle = ttk.Style()
        boldStyle.configure('Bold.TButton', font=('Arial', 14, 'bold'))
        self.streamButton = ttk.Button(self.mainTab, text='Stream',
                                       style='Bold.TButton', command=self.stream_command)
        self.streamButton.pack()

    
    def create_path_labels(self):
        self.workingDir = tk.StringVar(value='Initializing...')
        self.workingDirLabel = ttk.Label(self.mainTab, text=self.workingDir)
        self.workingDirLabel.pack()

        self.experimentDir = tk.StringVar(value='Initializing...')
        self.imageSaveFolderLabel = ttk.Label(self.mainTab, text=self.experimentDir)
        self.imageSaveFolderLabel.pack()
    
        
    def stream_command(self):
        if self.saveState.get() == 1:
            saveImageData = 'y'
        else:
            saveImageData = 'n'
        self.cliProcess.stdin.write('stream ' + self.streamMode.get() + ' ' + saveImageData + '\n')
        self.cliProcess.stdin.flush()
        print('Command sent to CLI')
        print('stream ' + self.streamMode.get() + ' ' + saveImageData + '\n')
        
        
    def enter_command(self, event):
        # handles commands sent through the GUI command line
        if self.cliInputText.get() == 'clc' or self.cliInputText.get() == 'clear': # clear history
            self.cliOutput.delete(0, tk.END)
        elif not self.cliInputText.get(): # prevents empty command from being sent
            return
        elif self.cliInputText.get() == 'exit':
            self.cliOutput.insert(tk.END, 'Please close the ToF GUI to exit the camera')
        else: # send message to CLI
            self.cliOutput.insert(tk.END, '>> ' + self.cliInputText.get())
            self.cliProcess.stdin.write(self.cliInputText.get() + '\n')
            self.cliProcess.stdin.flush()
            print('Command sent to CLI')
        self.cliInput.delete(0, tk.END) # clears the input box
        self.cliOutput.yview(tk.END) # moves the history box to the bottom
        self.update_vars()


    def update_vars(self):
        self.cliProcess.stdin.write('get_working_directory' + '\n')
        self.cliProcess.stdin.flush()
        self.workingDir.set(self.cliOutput.get(tk.END))

        self.cliProcess.stdin.write('get_experiment_name' + '\n')
        self.cliProcess.stdin.flush()
        self.experimentDir.set(self.cliOutput.get(tk.END))
        



            
    def read_CLI_output(self, event: threading.Event):
        # reads the stdout from the CLI and puts it in the GUI
        while True:
            output = self.cliProcess.stdout.readline()
            self.cliOutput.insert(tk.END, output)
            self.cliOutput.insert(tk.END, '')
            self.cliOutput.yview(tk.END) # moves the history box to the bottom
            
            # Catches CLI failing and closes GUI
            if not self.cliProcess.poll() is None: 
                self.root.destroy()
                break
            
            # Ends thread if user closes GUI
            if event.is_set():
                break

                
    def closing_protocol(self):
        if messagebox.askyesno(title='Quit?', message='Do you really want to exit the ToF Camera GUI?'):
            self.root.destroy()
            self.event.set()
            
            
    ### Webcam testing methods (can remove after integration with ToF camera)
    def stream_webcam(self):
        _, frame = self.cam.read()
        img = tk.Image.fromarray(cv.cvtColor(frame, cv.COLOR_BGR2RGBA))
        imgTk = tk.ImageTk.PhotoImage(image=img)
        self.imageLabel.imgTk = imgTk
        self.imageLabel.configure(image=imgTk)
        self.imageFrame.after(15, self.stream_webcam)
        

def main():
        cliProcess = subprocess.Popen(['python3', 'tof.py', '-gui'],
                                        shell=False, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, 
                                        start_new_session=True, bufsize=1, 
                                        universal_newlines=True) #as cliProcess:
        root = ToFGUI(cliProcess)
        root.root.mainloop()
        cliProcess.terminate()
    
if __name__ == '__main__':
    main()