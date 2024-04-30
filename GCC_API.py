import cmd
from datetime import datetime
from Espros_API import Espros_Commands as espros
import socket
import logging
import sys
import cv2
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import time
import platform


# ROS dependencies
if platform.system() == 'Linux':
    from ros_start import *

# enables autocomplete on Windows, cause apparently
# python doesn't ship with a version of readline that
# works on Windows
# further proof that Windows is a horrible OS
# and this year is the year of the Linux desktop
if platform.system() == 'Windows':
    from pyreadline3 import Readline
    readline = Readline()  # enables tab-autocomplete

# Define IPv4 address for the ToF camera
PORT = 50660
REMOTE_IP = '192.168.6.100'
buffer = 8192

# set up loggers
logging.basicConfig(format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s', datefmt='%Y-%m-%d %H:%M:%S',
                    filename='gcc_api.log', filemode='w', level=logging.DEBUG)
logger = logging.getLogger()
console_log = logging.StreamHandler(sys.stdout)
console_log.setLevel(logging.INFO)
logger.addHandler(console_log)

###############################################################################
# "global" Variables
###############################################################################
''' colormap link: https://www.analyticsvidhya.com/blog/2020/09/colormaps-matplotlib/
    (reference this site to choose your colormap)'''
CM = plt.get_cmap('jet')
cmap = mpl.cm.jet
backscatter_phasor = np.zeros((240, 320))  # default backscatter phasor
index_of_refraction = 1.334  # default index of refraction (for water)
mod_freq = 12e6
mod_freq_str = "12MHz"
speed_of_light = 2.998e8
working_dir = ""  # default image savepath
experiment_name = ""  # default experiment scene
is_corrected = False  # tracks whether backscatter subtraction is occuring
integration_time = int(13)  # default integration time
use_count_data = False  # determines data aquisition method
distance_colormap = "normalized"  # determines how the distance colormap is scaled
half_wavelength = speed_of_light / (2 * index_of_refraction * mod_freq)
max_distance_cm = half_wavelength * 100  #Theoretically, the maximum distance reading espros will ever give
min_distance_cm = 0 
user_max_distance_cm = max_distance_cm
user_min_distance_cm = 0
enable_red_box = False # puts a red box in the center of the image (used for distance testing)
enable_median_filter = False
median_filter_kernel = 5
enable_low_pass_filter = False
low_pass_filter_kernel = 5


# Default color mapping thresholds (img values are 8-bit)
cm_min_amp = 0
cm_max_amp = 255
cm_min_dist = 0 #can be deleted
cm_max_dist = 255 #can be deleted

#mpl.rcParams['interactive'] = False
font = {'family': 'arial', 'weight': 'bold', 'size': 20}
mpl.rc('font', **font)
plt.ioff()

class GCC_Commands(cmd.Cmd):
    logging.info("_GCC_TOF_CAM_ started successfully")
    intro = "Welcome to GCC ToF camera CLI (use help for list of commands)"
    prompt = "_GCC_TOF_CAM_: "
    ruler = ""

    ###########################################################################
    #                                ROS Stuff                                #
    ###########################################################################

    def preloop(self):
        try:
            ros_preloop()
        except NameError:
            # do nothing cause ROS code not here :(
            # bullying will continue until OS improves
            print("Say hi to Bill Gates for me, he's watching you though your webcam")

    def postloop(self):
        try:
            ros_postloop()
        except NameError:
            print("Yo, I put a backdoor in your HyperV")

    def do_ros(self, arg):
        "Streams amplify and distance image to ROS"
        try:
            stream_amp_and_dist_over_ROS()
        except NameError:
            print("ERROR: ROS is not installed, delete System32")

    def do_new_ros(self, arg):
        "Streams amplify and distance image to ROS"
        new_stream_amp_and_dist_over_ROS()

    def do_crab(self, arg):
        "crab time"
        try:
            crab()
        except NameError:
            print("the crabs are sad")
            print("you let them down")

    ###############################################################################
    # Backscatter Methods
    ###############################################################################

    # ======================= NHS

    def do_characterize_backscatter(self, arg):
        "Samples N images and averages and stores the image phasor as backscatter. Input is the number of images to be averaged as an integer"

        # Handling inputs
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Invalid argument amount, see help below:\n")
            return
        num_samples = int(args[0])

        global backscatter_phasor, is_corrected

        backscatter_phasor = np.zeros((240, 320))  # zeros the backscatter phasor before characterizing

        # Calculating wavelength in mm
        wavelength_mm = speed_of_light * 1000 / (index_of_refraction * mod_freq)

        # Initializing sums
        backscatter_amplitude = np.zeros((240, 320))
        backscatter_phase = np.zeros((240, 320))

        # Looping through desired number of samples and keeping a running average
        for i in range(0, num_samples):
            amplitude_data, phase_data = get_image('phasor_data', True)
            backscatter_amplitude += amplitude_data
            backscatter_phase += phase_data

        # Getting the average values by dividing through by the number of elements
        backscatter_amplitude = backscatter_amplitude / num_samples
        backscatter_phase = backscatter_phase / num_samples

        colored_image = CM(backscatter_amplitude.astype(int))
        print(colored_image)
        img = (colored_image[:, :, :3] * 255).astype(np.uint8)
        img = cv2.flip(img, 1)

        # Displaying subtracted image and waiting for user input to continue
        while True:
            cv2.imshow('Backscatter Amplitude Image - Press ''enter'' to exit', img)
            if cv2.waitKey(10) == 13:  # press enter to exit
                break

        cv2.destroyAllWindows()

        backscatter_phasor = backscatter_amplitude * np.exp(1j * backscatter_phase)
        is_corrected = True

    # ======================= NHS

    def do_reset_backscatter(self, arg):
        "Resets backscatter phasor to zero"
        global backscatter_phasor, is_corrected
        backscatter_phasor = np.zeros((240, 320))
        is_corrected = False

    # ======================= NHS

    def do_display_backscatter_phasor(self, arg):
        "Prints the current backscatter phasor and displays image"
        print(backscatter_phasor)

        # Getting phase and converting to distance
        # wavelength_mm = speed_of_light * 1000 / (index_of_refraction * mod_freq)
        # backscatter_phase = wrapTo2Pi(np.arctan2(backscatter_phasor.imag, backscatter_phasor.real))
        # img = (backscatter_phase * wavelength_mm / (4 * np.pi)).astype(int)

        # Getting amplitude and converting to image
        img = np.sqrt(backscatter_phasor.real ** 2 + backscatter_phasor.imag ** 2)

        colored_image = CM(img.astype(int))
        print(colored_image)
        img = (colored_image[:, :, :3] * 255).astype(np.uint8)
        img = cv2.flip(img, 1)

        # Displaying subtracted image and waiting for user input to continue
        while True:
            cv2.imshow('Backscatter Amplitude Image - Press ''enter'' to exit', img)
            if cv2.waitKey(10) == 13:  # press enter to exit
                break

        cv2.destroyAllWindows()

    ###############################################################################
    # Imaging Methods
    ###############################################################################

    # ======================= NHS

    def do_set_data_mode(self, arg):
        "Determines whether to use count data or DCS data, Input is either 'count' or 'DCS'"
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Too many arguments given, see help:\n")
            return
        if args[0] != 'count' and args[0] != 'dcs':
            logger.info("Incorrect data type. Must be either 'count' or 'dcs'")
            return
        global use_count_data
        if args[0] == 'count':
            use_count_data = True
            print("Using count data")
        else:
            use_count_data = False
            print("Using DCS data")

    # ======================= NHS

    def do_get_data_mode(self, arg):
        "Prints the current data collection mode"
        if use_count_data:
            print("Using count data")
        else:
            print("Using DCS data")

    # ======================= NHS

    def do_stream(self, arg):
        "Streams either amplitude or distance (specify) and saves images on spacebar keypress. Input is 'amplitude' or 'distance'"

        save_image_data = False

        # Ensuring there is a working directory to put images into
        img_savepath = check_and_get_directory()

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) > 2:
            logger.info("Invalid argument amount, see help\n")
            return

        stream_type = args[0]
        if (stream_type != 'amplitude' and stream_type != 'a' and stream_type != 'distance' and stream_type != 'd'):
            logger.info("Invalid argument - must be either 'amplitude' or 'distance'")
            return

        if len(args) == 1:
            while True:
                userInput = input('Do you wish to save raw image data when saving images? Enter ''yes'' or ''no'': ')
                if userInput == 'yes' or userInput == 'YES' or userInput == 'Yes':
                    save_image_data = True
                    break
                elif userInput == 'no' or userInput == 'NO' or userInput == 'No':
                    save_image_data = False
                    break
                else:
                    print("Unrecognized input. Must be 'yes' or 'no'")
        else:
            if args[1] == 'y':
                save_image_data = True
            else:
                save_image_data = False
            
        # Creating a window to display the stream
        cv2.namedWindow('Stream - Press ''enter'' to exit', cv2.WINDOW_NORMAL) # set to cv2.WINDOW_NORMAL?
        cv2.setWindowProperty('Stream - Press ''enter'' to exit', cv2.WND_PROP_TOPMOST, 1)
        print('Stream started - Press ''enter'' to exit'' or press ''spacebar'' to save an image')

        while True:
            if save_image_data:
                img_amp_data, img_dist_data, pre_img_amp_data, pre_img_dist_data = get_image('both', True)
            else:
                img_amp_data, img_dist_data = get_image('mcdobe_sandstorm', False)
                

            # Converting amplitude image to color
            img_amp = rescale_amplitude_colormap(img_amp_data)
            colored_image_amp = CM(img_amp)
            img_amp = (colored_image_amp[:, :, :3] * 255).astype(np.uint8)
            img_amp = cv2.flip(img_amp, 1)

            # Converting distance image to color
            img_dist = rescale_distance_colormap(img_dist_data)
            colored_image_dist = CM(img_dist)
            img_dist = (colored_image_dist[:, :, :3] * 255).astype(np.uint8)
            img_dist = cv2.flip(img_dist, 1)
            
            # Applying a median filter
            if enable_median_filter:
                img_amp = cv2.medianBlur(img_amp, median_filter_kernel)
                img_dist = cv2.medianBlur(img_dist, median_filter_kernel)
            
            if enable_low_pass_filter:
                img_amp = cv2.GaussianBlur(img_amp, (low_pass_filter_kernel, low_pass_filter_kernel), 0)
                img_dist = cv2.GaussianBlur(img_dist, (low_pass_filter_kernel, low_pass_filter_kernel), 0)
            

            if stream_type == 'amplitude' or stream_type == 'a':  # creating amplitude window
                cv2.imshow('Stream - Press ''enter'' to exit', img_amp)
                key = cv2.waitKey(10)
                if key == 13:  # press enter to exit
                    # cv2.imwrite('img.jpg', img_amp)
                    break
            else:  # creating distance window
            
                # Creating distance colorbar
                fig, ax = plt.subplots(figsize = (4, 4), dpi = 60)
                norm = mpl.colors.Normalize(vmin = user_min_distance_cm, vmax = user_max_distance_cm)
                fig.colorbar(mpl.cm.ScalarMappable(norm = norm, cmap = cmap),
                             cax = ax, orientation = 'vertical', label = 'Distance (cm)')
                fig.tight_layout(pad = 0)
                ax.margins(0)

                fig.canvas.draw()
                width, height = fig.canvas.get_width_height()
                image_from_plot = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(height, width, 3)
                
                # Appending colorbar to image
                white_bar =  255*np.ones((240,20,3),dtype=np.uint8)
                img_dist = np.concatenate((img_dist, white_bar, image_from_plot[:,180:240,:]), axis=1)
                
                if enable_red_box:
                    square_width = 10
                    start_point = (int(180 - square_width/2), int(120 - square_width/2))
                    end_point = (int(180 + square_width/2), int(120 + square_width/2))
                    color = (0, 0, 255)
                    thickness = 2
                    img_dist = cv2.rectangle(img_dist, start_point, end_point, color, thickness)
                    
                cv2.imshow('Stream - Press ''enter'' to exit', img_dist)
                key = cv2.waitKey(10)
                if key == 13:  # press enter to exit
                    break
                plt.close(fig) # closing the colorbar figure

            # Saving image if spacebar is pressed
            if key == 32:
                currDateTime = datetime.now().strftime("%Y%m%d-%H%M%S")
                # Saving amplitude image
                img_name = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_amp.jpg"
                filepath = os.path.join(img_savepath, img_name)
                if not cv2.imwrite(filepath, img_amp):
                    logging.warning("Image save failed - ensure output directory exists")

                # Saving distance image
                img_name = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_dist.jpg"
                filepath = os.path.join(img_savepath, img_name)
                if not cv2.imwrite(filepath, img_dist):
                    logging.warning("Image save failed - ensure output directory exists")

                print("Yo dawg boss - I took a picture!")

                # Checking to see if image data should be saved
                if save_image_data:
                    # Saving backscatter subtracted amplitude image data
                    filename = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_amp_data.csv"
                    filepath = os.path.join(img_savepath, filename)
                    np.savetxt(filepath, img_amp_data, delimiter=',')

                    # Saving backscatter subtracted distance image data
                    filename = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_dist_data.csv"
                    filepath = os.path.join(img_savepath, filename)
                    np.savetxt(filepath, img_dist_data, delimiter=',')

                    # Saving unprocessed amplitude image data
                    filename = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_amp_pre_data.csv"
                    filepath = os.path.join(img_savepath, filename)
                    np.savetxt(filepath, pre_img_amp_data, delimiter=',')

                    # Saving unprocessed distance image data
                    filename = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_dist_pre_data.csv"
                    filepath = os.path.join(img_savepath, filename)
                    np.savetxt(filepath, pre_img_dist_data, delimiter=',')

                    # Saving backscatter subtraction phasor
                    filename = currDateTime + "_" + mod_freq_str + str(integration_time) + "us_backscatter_data.csv"
                    filepath = os.path.join(img_savepath, filename)
                    np.savetxt(filepath, backscatter_phasor, delimiter=',')

        cv2.destroyAllWindows()

    ###############################################################################
    # Illuminator Methods
    ###############################################################################

    # =======================

    # Added by Alex
    def do_disable_illuminators(self, arg):
        "Disables camera illuminators"
        s = build_tcp_connection()
        s.sendall(bytes(('enableIllumination 00' + '\n').encode('ascii')))
        print("Turned off Illuminators")

    def help_disable_illuminators(self):
        logger.info("Turns off the stock IR illuminators")

    # ======================= NHS

    def do_enable_illuminators(self, arg):
        "Enables camera illuminators"
        s = build_tcp_connection()
        s.sendall(bytes(('enableIllumination 01' + '\n').encode('ascii')))
        print("Turned on illuminators")

    def help_enable_illuminators(self):
        logger.info("Turns on the stock IR illuminators")

    ###############################################################################
    # Integration Time Methods
    ###############################################################################

    # =======================

    # Added by Alex
    def do_set_integration_time(self, arg):
        "Changes integration time"

        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Invalid argument amount, see help below:\n")
            return

        global integration_time

        int_time = arg_to_argv(arg)
        s = build_tcp_connection()
        if int(int_time[0]) <= 4000:
            print("Integration time set to", int_time[0])
            s.sendall(bytes(('setIntegrationTime3D ' + str(int_time[0]) + '\n').encode('ascii')))
            integration_time = int(int_time[0])
        elif int(int_time[0]) < 9000:
            print("Integration time set to 6553 us")
            integration_time = 6553
            writeIntegrationRegisters(s, '00', '08', 'FF', 'FB')
        elif int(int_time[0]) < 18000:
            print("Integration time set to 13100 us")
            writeIntegrationRegisters(s, '00', '10', 'FF', 'FB')
            integration_time = 13100
        elif int(int_time[0]) < 30000:
            print("Integration time set to 26200 us")
            writeIntegrationRegisters(s, '00', '20', 'FF', 'FB')
            integration_time = 26200
        elif int(int_time[0]) < 60000:
            print("Integration time set to 52426 us")
            writeIntegrationRegisters(s, '00', '40', 'FF', 'FB')
            integration_time = 52425
        return

    def do_get_integration_time(self, arg):
        "Prints out the current integration time"
        print(integration_time)
        return

    ###############################################################################
    # Index of Refraction Methods
    ###############################################################################

    # ======================= NHS

    def do_set_index_of_refraction(self, arg):
        "Sets the index of refraction to the specified value. Input is the index of refraction as an integer"
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Invalid argument amount, see help below:\n")
            return
        global index_of_refraction, half_wavelength
        index_of_refraction = float(args[0])
        half_wavelength = speed_of_light / (2 * index_of_refraction * mod_freq)
        print("Index of refraction changed to", index_of_refraction)

    # ======================= NHS

    def do_get_index_of_refraction(self, arg):
        "Prints the current index of refraction"
        print("The current index of refraction is", index_of_refraction)


    # ======================== ADB

    def do_restore_defaults(self, arg):
        "Sets integration time, modulation frequency,and index of refraction to their default values for water"

        # Modulation frequency = 12 MHz
        global mod_freq, mod_freq_str, half_wavelength, max_distance_cm
        sock = build_tcp_connection()
        espros.do_setModulationFrequency(sock, '1')
        mod_freq = 12e6
        mod_freq_str = "12MHz"

        # Index of refraction
        global index_of_refraction
        index_of_refraction = 1 #air for testing
        half_wavelength = speed_of_light / (2 * index_of_refraction * mod_freq)
        max_distance_cm = half_wavelength * 100

        # Integration time = 100 us
        global  integration_time
        s = build_tcp_connection()
        print("Integration time set to", str(100))
        s.sendall(bytes(('setIntegrationTime3D ' + str(100) + '\n').encode('ascii')))
        integration_time = 100



    ###############################################################################
    # Modulation Frequency Methods
    ###############################################################################

    # ======================= Written and Directed by Adobe Sandstorm

    def do_set_modulation_frequency(self, arg):
        'Sets the modulation frequency of the camera. Usage Example: 24 MHz should be entered as "24"'
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Too many arguments given, see help:\n")
            return

        global mod_freq, mod_freq_str, half_wavelength, max_distance_cm
        sock = build_tcp_connection()
        if args[0] == '24':
            espros.do_setModulationFrequency(sock, '0')
            mod_freq = 24e6
            mod_freq_str = "24MHz"
        elif args[0] == '12':
            espros.do_setModulationFrequency(sock, '1')
            mod_freq = 12e6
            mod_freq_str = "12MHz"
        elif args[0] == '6':
            espros.do_setModulationFrequency(sock, '2')
            mod_freq = 6e6
            mod_freq_str = "6MHz"
        elif args[0] == '3':
            espros.do_setModulationFrequency(sock, '3')
            mod_freq = 3e6
            mod_freq_str = "3MHz"
        elif args[0] == '1.5':
            espros.do_setModulationFrequency(sock, '4')
            mod_freq = 1.5e6
            mod_freq_str = "1500KHz"
        elif args[0] == '750KHz':
            espros.do_setModulationFrequency(sock, '5')
            mod_freq = 750e3
            mod_freq_str = "750KHz"

        else:
            print("Please enter a valid modulation frequency")
            return

        # Updating the half-wavelength
        half_wavelength = speed_of_light / (2 * index_of_refraction * mod_freq)
        print("Modulation frequency changed to " + mod_freq_str)
        max_distance_cm = half_wavelength * 100
        return

    # ======================= NHS

    def do_get_modulation_frequency(self, arg):
        "Prints the current modulation frequency"
        print("The current modulation frequency is", mod_freq, "Hz")

    ###############################################################################
    # File Management Methods
    ###############################################################################

    # ======================== NHS

    def do_set_working_directory(self, arg):
        "Sets the working directory to the specified filepath"

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Invalid argument amount, see help\n")
            return

        global working_dir
        working_dir = args[0]

    # ======================== NHS

    def do_get_working_directory(self, arg):
        "Displays the current working directory"
        print(working_dir)

    # ======================== NHS

    def do_set_experiment_name(self, arg):
        "Sets the experiment name (folder in working directory where images will be saved)"

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Invalid argument amount, see help\n")
            return

        global experiment_name
        experiment_name = args[0]

    # ======================== NHS

    def do_get_experiment_name(self, arg):
        "Displays the current experiment name"
        print(experiment_name)

    ###############################################################################
    # Register Methods
    ###############################################################################

    # ======================= ADB

    def do_read_register(self, arg):
        "Returns the register contents of the given register"

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) != 1:
            logger.info("Invalid argument amount - expected 1 arguments, see help\n")
            return
        addr = args[0]

        sock = build_tcp_connection()
        espros.do_readRegister(sock, addr)
        return

        # ======================= ADB

    def do_write_register(self, arg):
        "Writes the given value to the given register (Expected args: Address Value)"

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) != 2:
            logger.info("Invalid argument amount - expected 2 arguments, see help\n")
            return
        addr = args[0]
        value = args[1]

        sock = build_tcp_connection()
        writeRegister(sock, addr, value)
        return

        # ======================= ADB

    def do_dump_registers(self, arg):
        "Prints out all readable register values"

        for i in range(0, 256):
            hex_str = hex(i)[2:] # remove '0x' at front
            hex_str_padded = "0" * (2-len(hex_str)) + hex_str # pad with extra zero if necessary
            sock = build_tcp_connection()
            espros.do_readRegister(sock, hex_str_padded)

        return

    ###############################################################################
    # Colormap Methods
    ###############################################################################

    # ======================= ADB

    def do_set_amplitude_colormap(self, arg):
        "Sets the amplitude colormap min and max values. Inputs are the min and max values, space-delimited"

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) != 2:
            logger.info("Invalid argument amount - expected 2 arguments, see help\n")
            return
        min_val = int(args[0])
        max_val = int(args[1])

        # Checking the validity of the inputs
        if min_val < 0:
            logger.info("Invalid minimum value. Must be greater than 0")
            return
        elif max_val > 255:
            logger.info("Invalid maximum value. Must be less than 255")
            return
        elif min_val > max_val:
            logger.info("Invalid values. Maximum value must be greater than the minimum value")
            return

        global cm_min_amp, cm_max_amp
        cm_min_amp = min_val
        cm_max_amp = max_val
        return

    # ======================= NHS

    def do_get_colormap_maxima(self, arg):
        "Prints the colormap min and max values"
        print("The colormap maxima are: ")
        print("     Amplitude minimum:", cm_min_amp)
        print("     Amplitude maximum:", cm_max_amp)
        print("     Distance minimum (in cm):", user_min_distance_cm)
        print("     Distance maximum (in cm):", user_max_distance_cm)

        # ======================= NHS

    def do_reset_amplitude_colormap(self, arg):
        "Resets the amplitude colormap min and max values to the defaults (0, 255)"
        global cm_min_amp, cm_max_amp
        cm_min_amp = int(0)
        cm_max_amp = int(255)

    # ======================= NHS

    def do_reset_distance_colormap(self, arg):
        "Resets the distance colormap min and max values to the defaults (0, 255)"
        global cm_min_dist, cm_max_dist
        cm_min_dist = int(0)
        cm_max_dist = int(255)

        # ====================== ADB

    def do_set_distance_colormap(self, arg):
        "Sets the distance colormap min and max values. Inputs are the min and max values as integers, space-delimited"

        # Parsing inputs
        args = arg_to_argv(arg)
        if len(args) != 2:
            logger.info("Invalid argument amount, see help\n")
            return
        min_val = int(args[0])
        max_val = int(args[1])

        # Checking the validity of the inputs
        if min_val < 0:
            logger.info("Invalid minimum value. Must be greater than 0")
            return
        elif max_val > 255:
            logger.info("Invalid maximum value. Must be less than 255")
            return
        elif min_val > max_val:
            logger.info("Invalid values. Maximum value must be greater than the minimum value")
            return

        global cm_min_dist, cm_max_dist
        cm_min_dist = int(args[0])
        cm_max_dist = int(args[1])
        return

    # ======================= NHS (dev in progress, not currently being used)

    def do_set_distance_colormap(self, arg):
        "Sets whether the distance image colormap is according to absolute distance or normalized distance"
        args = arg_to_argv(arg)

        global distance_colormap
        if args[0] == 'normalized' or args[0] == 'n' or args[0] == 'N':
            distance_colormap = 'normalized'
        elif args[0] == 'absolute' or args[0] == 'a' or args[0] == 'A':
            distance_colormap = 'absolute'
        else:
            print("Invalid arguments. Must be either 'absolute' or 'normalized'")


    # ======================= ADB

    def do_set_max_distance_cm(self, arg):
        "Sets the maximum imaging distance in cm"
        global max_distance_cm, user_max_distance_cm, min_distance_cm, cm_max_dist
        args = arg_to_argv(arg)

        if len(args) != 1:
            logger.info("Invalid argument amount, see help\n")
            return

        user_val = int(args[0])
        if user_val <= max_distance_cm and user_val >= min_distance_cm:
            user_max_distance_cm = user_val
            cm_max_dist = int((user_max_distance_cm/max_distance_cm)*255)
        else:
            print("Invalid arguments. Must be greater than 0 and less than ", str(max_distance_cm))

    # ======================= ADB

    def do_set_min_distance_cm(self, arg):
        "Sets the maximum imaging distance in cm"
        global max_distance_cm, user_min_distance_cm, min_distance_cm, cm_min_dist
        args = arg_to_argv(arg)

        if len(args) != 1:
            logger.info("Invalid argument amount, see help\n")
            return

        user_val = int(args[0])
        if user_val <= max_distance_cm and user_val >= min_distance_cm:
            user_min_distance_cm = user_val
            cm_min_dist = int((user_min_distance_cm / max_distance_cm) * 255)
        else:
            print("Invalid arguments. Must be greater than 0 and less than ", str(max_distance_cm))

    ###############################################################################
    
    # ======================= NHS
    
    def do_toggle_red_box(self, arg):
        "Toggles whether a red box will appear in the center of the image stream (primarily used for testing). No input arguments required"
        global enable_red_box
        if enable_red_box:
            enable_red_box = False
        else:
            enable_red_box = True
            
    ###############################################################################
    # Filtering Methods
    ###############################################################################
            
    # ======================= NHS
            
    def do_toggle_median_filter(self, arg):
        "Toggles the median filter on or off depending on current state. No input arguments required"
        global enable_median_filter
        if enable_median_filter:
            enable_median_filter = False
            print('Median filtering has been turned off')
        else:
            enable_median_filter = True
            print('Median filtering has been turned on')
            
    # ======================= NHS
    
    def do_set_median_filter_kernel(self, arg):
        "Sets the size of the median filter kernel size (assuming a square kernel). Expected arguments: kernel size as a positive, odd integer"
        global median_filter_kernel
        args = arg_to_argv(arg)
        user_inp = int(args[0])
        if user_inp % 2 == 0:
            print('Kernel size must be an odd integer')
            return
        if user_inp < 0:
            print('Kernel size must be positive')
            return
        median_filter_kernel = user_inp
        print('Median filter kernel size set to ', median_filter_kernel)
    
    # ======================= NHS
    
    def do_toggle_low_pass_filter(self, arg):
        "Toggles the low pass (Gaussian blur) filter on or off depending on current state. No input arguments required"
        global enable_low_pass_filter
        if enable_low_pass_filter:
            enable_low_pass_filter = False
            print('Low pass filtering has been turned off')
        else:
            enable_low_pass_filter = True
            print('Low pass filtering has been turned on')
    
    # ======================= NHS
    
    def do_set_low_pass_filter_kernel(self, arg):
        "Sets the size of the median filter kernel size (assuming a square kernel). Expected arguments: kernel size as a positive, odd integer"
        global low_pass_filter_kernel
        args = arg_to_argv(arg)
        user_inp = int(args[0])
        if user_inp % 2 == 0:
            print('Kernel size must be an odd integer')
            return
        if user_inp < 0:
            print('Kernel size must be positive')
            return
        low_pass_filter_kernel = user_inp
        print('Low pass filter kernel size set to ', low_pass_filter_kernel)
        
    # ======================= NHS
    
    def do_get_class_methods(self):
        commands = dir(self)
        print(commands)
        
    # ======================= NHS
    
    def do_temperature_test(self, arg):
        'Test method to characterize temperature behavior - don''t run due to filepath dependencies'
        
        password = input('Password: ')
        if not password == 'abcdefg':
            print('Incorrect password')
            return
        
        totalTime_min = 45
        timeStep_sec = 15
        numFrames = 5
        
        N = int(totalTime_min*60/timeStep_sec) + 1
        dist = np.zeros(N)
        timeVals = np.arange(0, totalTime_min + timeStep_sec/60, step=timeStep_sec/60)
        
        for i in range(N):
            distVals = np.zeros(5)
            for j in range(numFrames):
                imgDist = get_image('distance', False)
                distVals[j] = np.mean(imgDist[115:124, 175:184])
            dist[i] = np.mean(distVals)
            print(i*timeStep_sec/60, 'minutes elapsed')
            time.sleep(timeStep_sec)
            
        
        np.savetxt('C:/Users/STUYCKNH19/OneDrive - Grove City College/Documents/GCC/Research/time_data.txt', 
                   timeVals, delimiter=',')
        np.savetxt('C:/Users/STUYCKNH19/OneDrive - Grove City College/Documents/GCC/Research/dist_data.txt', 
                   dist, delimiter=',')
        
        
    # Exit Method
    ###############################################################################

    # =======================

    def do_exit(self, arg):
        "Exit the program"
        logging.info("_GCC_TOF_CAM_ shut down successfully")
        return True
    
    
    def do_blah(self, arg):
        args = arg_to_argv(arg)
        count = 0
        for i in range(10):
            print(i)
        time.sleep(5)
        
    def do_write_something(self, arg):
        file = open('write_to_me.txt', 'a')
        file.write('hello there' + '\n')
        file.close()
        print('wrote something')


# End of class


# =======================
# local/private functions
# =======================

def build_tcp_connection():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
    logging.debug('Socket created')

    try:
        s.connect((REMOTE_IP, PORT))
        logging.debug('Connection successful')
    except:
        logging.warning('Socket connection failed')
    return s


def mp4_init(fp, framerate=15, x=320, y=240):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(fp, fourcc, int(framerate), (x, y))
    return out


def arg_to_argv(arg):
    return arg.split()


def generate_dynamic_framerate(filepath):
    framerate = 10
    # TODO: get start and end times of pictures in filepath
    # TODO: calculate the framerate
    return framerate


def get_image(image_type, return_pre_subtracted_img):
    '''
    Returns background subtracted image by specified type. Note: this method
    will return a different number of outputs depending on the specified inputs
    Inputs:
        image_type - string/character array specifying the outputs. Options
            are: 'amplitude', 'distance', 'both'
            subtracted image data should be returned
        return_pre_subtracted_img - boolean indicating whether the pre-backscatter
            subtracted image data data should be returned
    Outputs:
        img_amp - post-backscatter subtracted amplitude image
        img_dist - post-backscatter subtracted distance image
        img_amp_pre - pre-backscatter subtracted amplitude image
        img_dist_pre - pre-backscatter subtracted distance image
    '''

    # For each frame getting data from camera
    wavelength = speed_of_light / (index_of_refraction * mod_freq)
    if use_count_data:  # uses count data
        sock = build_tcp_connection()
        amp = espros.do_getAmplitudeSorted(sock, 'raw')
        sock = build_tcp_connection()
        count_data = espros.do_getDistanceSorted(sock, 'raw')
        dist = (wavelength / 2) * (count_data / 30000.0) * 100  # distance array in cm
        phase = count_data * 2 * np.pi / 30000.0
    else:  # uses DCS data
        sock = build_tcp_connection()
        dcs0, dcs1, dcs2, dcs3 = espros.do_getDCSSorted(sock, 'raw')

        # Calculating amplitude and phase from DCS
        amp = np.sqrt(((dcs3 - dcs1) / 2) ** 2 + ((dcs2 - dcs0) / 2) ** 2)
        amp = np.nan_to_num(amp)  # converting NaN's to zeros
        phase = np.pi + np.arctan2((dcs3 - dcs1), (dcs2 - dcs0))
        dist = (wavelength / 2) * phase * 100 / (2 * np.pi)  # distance in cm
        dist = np.nan_to_num(dist)
        print("dist pre backscatter")
        print(dist[120][180])
        print("\n")

    # Returning amplitude and phase data
    if image_type == 'phasor_data':
        return amp, phase

    # Calculating observed phasor
    observed_phasor = amp * np.exp(1j * phase)

    # Implement background subtraction
    object_phasor = observed_phasor - backscatter_phasor
    imgAmp = np.sqrt(object_phasor.real ** 2 + object_phasor.imag ** 2)
    imgAmp = np.nan_to_num(imgAmp)  # converting NaN's to zeros

    object_phase = wrapTo2Pi(np.arctan2(object_phasor.imag, object_phasor.real))
    imgDist = object_phase * (wavelength / 2) * 100 / (2 * np.pi)  # backscatter subtracted distance array in cm
    imgDist = np.nan_to_num(imgDist)  # converting NaN's to zeros

    # Return based on specified data type
    if image_type == 'amplitude':
        if return_pre_subtracted_img:
            return imgAmp, amp, dist
        else:
            return imgAmp
    elif image_type == 'distance':
        if return_pre_subtracted_img:
            return imgDist, amp, dist
        else:
            return imgDist
    else:
        if return_pre_subtracted_img:
            return imgAmp, imgDist, amp, dist
        else:
            return imgAmp, imgDist


def rescale_amplitude_colormap(img):
    # Implements the colormap cutoffs
    img[img > cm_max_amp] = cm_max_amp
    img[img < cm_min_amp] = cm_min_amp

    # Rescale from [0,255]
    img = (255.0 * (img - cm_min_amp) / float(cm_max_amp)).astype(int)
    return img


def rescale_distance_colormap(img):
    # implements the colormap cutoffs

    #limit the distance values to be within the user's given range
    img[img > user_max_distance_cm] = user_max_distance_cm
    img[img < user_min_distance_cm] = user_min_distance_cm

    #rescales from user range to 0-255 for colormap
    img = (255.0 * (img - user_min_distance_cm) / float(user_max_distance_cm)).astype(int)
    return img


def wrapTo2Pi_scalar(number):
    if number < 0:
        number = 2 * np.pi + number
    return number


def wrapTo2Pi(angles_array):
    for idxRow in range(0, 240):
        for idxCol in range(0, 320):
            if angles_array[idxRow][idxCol] < 0:
                angles_array[idxRow][idxCol] = 2 * np.pi + angles_array[idxRow][idxCol]
    return angles_array


def writeIntegrationRegisters(s, a0, a1, a2, a3):
    writeRegister(s, 'a0', a0)
    writeRegister(s, 'a1', a1)
    writeRegister(s, 'a2', a2)
    writeRegister(s, 'a3', a3)
    return


def writeRegister(s, register, value):
    s = build_tcp_connection()
    s.sendall(bytes(('w ' + str(register) + ' ' + str(value) + '\n').encode('ascii')))
    return


def check_and_get_directory():
    # Checks to see if the current output directory exists, if not, creates it
    if working_dir and experiment_name:
        file_path = working_dir + "\\" + experiment_name
    elif not working_dir and not experiment_name:  # if both working_dir and experiment_name are empty
        return ""
    elif not working_dir and experiment_name:  # if working_dir is empty and experiment_name is a path
        file_path = experiment_name
    elif working_dir and not experiment_name:  # if working dir is a path and experiment_name is empty
        file_path = working_dir
    else:
        logger.info("Unknown filepath. Images will be saved to the directory where the run scripts are located")

    # Checking that the directory exists
    if os.path.isdir(file_path):
        return file_path
    else:
        os.mkdir(file_path)
        return file_path
        return working_dir + "\\" + experiment_name
