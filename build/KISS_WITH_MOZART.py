import FreeSimpleGUI as sg
# import PySimpleGUIQt as sg
import os.path
import PIL.Image
import io
import base64

import cv2
import numpy
import math

import serial
import time
from datetime import datetime

baudrates =  ['115200', '230400', '460800', '921600']
mode_names = ['OFF', 'SD', 'ZOOM', 'HD']
error_type = ['Unknown', 'Invalid cmd', 'Invalid param', 'Wrong mode', 'Image sensor']

# ---------------------------------Image Processing-------------------------------
# Raw data processing
def parse_raw_file_to_image(
    path: str,
    #x_size: int = 640,
    #y_size: int = 480,
    x_size: int = 64,
    y_size: int = 48,

    #line_total_bytes: int = 648,
    line_total_bytes: int = 72,
    drop_head: int = 5,
    drop_tail: int = 1,
    expect_line_prefix_bytes: int = 2,   # LN1 + LN2
    validate_line_number: bool = False,  
    rgb: bool = True                     
) -> PIL.Image.Image:
    """
    Parse a raw capture file of size 648*480.
    For each 648-byte line:
      - drop first 5 bytes and last 1 byte -> 642 bytes remain
      - first 2 bytes of the 642 are line number (LN1, LN2)
      - remaining 640 bytes are grayscale pixels for that row
    Returns a PIL Image.
    """
    expected_file_size = line_total_bytes * y_size
    usable_bytes = line_total_bytes - drop_head - drop_tail  # 642
    expected_usable = expect_line_prefix_bytes + x_size      # 2 + 640 = 642

    if usable_bytes != expected_usable:
        raise ValueError(
            f"Config mismatch: usable_bytes={usable_bytes} but expected {expected_usable} "
            f"(prefix {expect_line_prefix_bytes} + x_size {x_size})"
        )

    with open(path, "rb") as f:
        data = f.read()

    if len(data) != expected_file_size:
        raise ValueError(
            f"Unexpected file size: got {len(data)} bytes, expected {expected_file_size} "
            f"({line_total_bytes}*{y_size})."
        )

    # Create output image
    if rgb:
        img = PIL.Image.new("RGB", (x_size, y_size), color="black")
        pixels = img.load()
    else:
        img = PIL.Image.new("L", (x_size, y_size), color=0)
        pixels = img.load()

    # Parse each line
    for y in range(y_size):
        start = y * line_total_bytes
        line = data[start : start + line_total_bytes]

        # Strip header/footer inside the 648-byte line
        payload = line[drop_head : line_total_bytes - drop_tail]  # 642 bytes

        # Extract line number (optional)
        ln1, ln2 = payload[0], payload[1]
        line_no = (ln1 << 8) | ln2

        # Extract 640 pixels
        row = payload[expect_line_prefix_bytes : expect_line_prefix_bytes + x_size]

        if validate_line_number:
            if not (0 <= line_no < 65536):
                raise ValueError(f"Invalid line number at y={y}: {line_no}")

        if rgb:
            for x in range(x_size):
                dat = row[x]
                pixels[x, y] = (dat, dat, dat)
        else:
            for x in range(x_size):
                pixels[x, y] = row[x]

    return img

# Gamma adjustment
def adjust_gamma(img, gamma=1.0):
  invGamma = 1.0 / gamma
  table = numpy.array([((i / 255.0) ** invGamma) * 255
    for i in numpy.arange(0, 256)]).astype("uint8")
  return cv2.LUT(img, table)

# Contrast Limited Adaptive Histogram Equalization (CLAHE)
def apply_clahe(img):
  clahe = cv2.createCLAHE(clipLimit=1, tileGridSize=(8,8))
  lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
  l, a, b = cv2.split(lab)  # split on 3 different channels
  l2 = clahe.apply(l)  # apply CLAHE to the L-channel
  lab = cv2.merge((l2,a,b))  # merge channels
  result = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
  return result

# White Balance
def white_balance(img):
  result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
  avg_a = numpy.average(result[:, :, 1])
  avg_b = numpy.average(result[:, :, 2])
  result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
  result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
  result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
  return result

# Mask for simplest color balance
def apply_mask(matrix, mask, fill_value):
  masked = numpy.ma.array(matrix, mask=mask, fill_value=fill_value)
  return masked.filled()

# Threshold for simplest color balance
def apply_threshold(matrix, low_value, high_value):
  low_mask = matrix < low_value
  matrix = apply_mask(matrix, low_mask, low_value)
  high_mask = matrix > high_value
  matrix = apply_mask(matrix, high_mask, high_value)
  return matrix

# Simplest Color Balance Algorithm
def simplest_cb(img, percent):
  assert img.shape[2] == 3
  assert percent > 0 and percent < 100
  half_percent = percent / 200.0
  channels = cv2.split(img)
  out_channels = []
  for channel in channels:
    assert len(channel.shape) == 2
    height, width = channel.shape
    vec_size = width * height
    flat = channel.reshape(vec_size)
    assert len(flat.shape) == 1
    flat = numpy.sort(flat)
    n_cols = flat.shape[0]
    low_val  = flat[int(math.floor(n_cols * half_percent))]
    high_val = flat[int(math.ceil( n_cols * (1.0 - half_percent)))]
    thresholded = apply_threshold(channel, low_val, high_val)
    normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
    out_channels.append(normalized)
  return cv2.merge(out_channels)

# Channel scaling
def scale_chn(img, chn, factor):
  img = img.astype('uint16')
  scale = int(2**(factor-1))
  if chn == 'r':
    b = img[:,:,0]
    g = img[:,:,1]
    r = numpy.clip(img[:,:,2]+scale-1, 0, 255)
  if chn == 'g':
    b = img[:,:,0]
    g = numpy.clip(img[:,:,1]+scale-1, 0, 255)
    r = img[:,:,2]
  if chn == 'b':
    b = numpy.clip(img[:,:,0]+scale-1, 0, 255)
    g = img[:,:,1]
    r = img[:,:,2]
 
  return cv2.merge((b.astype('uint8'), g.astype('uint8'), r.astype('uint8'))) 

# --------------------------------Camera Interface --------------------------------


def wait_for_response(data_size = 2):
    startchar = int.from_bytes( s.read(1), "big")
    if startchar == 0x40:
        raw_bytes = s.read(3+data_size+1)
        #print(len(raw_bytes))
        if len(raw_bytes) == 3+data_size+1:
            command =  raw_bytes[0]
            result = raw_bytes[2]
            data = raw_bytes[3:data_size+3]
            endchar = raw_bytes[3+data_size]
            if endchar == 0x0D:
                #print("Reply Received: cmd [" + str(command) + "], result [" + str(result) + "]")
                return [command, result, data]

def wait_for_telem():
    startchar = int.from_bytes( s.read(1), "big")
    if startchar == 0x40:
      raw_bytes = s.read(4)
      append_output_window("Received Header:") # append for debugging
      append_output_window(str(raw_bytes)) # append for debugging
      if len(raw_bytes) != 4:
        append_output_window('Timed out waiting for header.')
        return []
      
      command = raw_bytes[0]
      mode = raw_bytes[1]
      window['-CAMSTATE-'].update(mode_names[mode])
      window['-MODE-'].update(set_to_index = mode)
        
      telem_length = int((raw_bytes[2]<<8) + raw_bytes[3])
      append_output_window("Data payload length:") # append for debugging
      append_output_window(str(telem_length)) # append for debugging
      telemetry = s.read(telem_length) # Data payload without terminate byte `0x0D`
      if len(telemetry) != telem_length:
        append_output_window('Timed out waiting for reply.')
        return []        
          
      #error response
      if command == 0xFF:
        append_output_window('Error Response: ' + error_type[telemetry[1]] + ' to command: ' + str(telemetry[0]))
        return []
            
      #ack  
      else:
        if len(telemetry) < 10:
          append_output_window('Good Response Received: ' + str(telemetry))
        else:
          #append_output_window('Good Response Received: ' + str(len(telemetry)) + ' bytes.')
          pass
            
        return telemetry # If download, return bytes array is `LN1 + LN2 + IMG`
      
    else:
      append_output_window('Timed out waiting for start character.')
      return []
        
     

def convert_to_bytes(file_or_bytes):

    #Will convert into bytes an image that is a file or a base64 bytes object.
    img = PIL.Image.open(file_or_bytes)

    with io.BytesIO() as bio:
        img.save(bio, format="PNG")
        del img
        return bio.getvalue()
      

def LEDIndicator(key=None, radius=30):
    return sg.Graph(canvas_size=(radius, radius),
             graph_bottom_left=(-radius, -radius),
             graph_top_right=(radius, radius),
             pad=(0, 0), key=key)

def SetLED(window, key, color):
    graph = window[key]
    graph.erase()
    graph.draw_circle((0, 0), 12, fill_color=color, line_color=color)

def append_output_window(text):
  window['-OUT-'].update(window['-OUT-'].get() + '\n' + text)


# --------------------------------- Define Layout ---------------------------------

# First the window layout...2 columns

sg.theme('BrownBlue')

comms_col = [[sg.Text('Comms Status:'), sg.Push(), sg.Text('Not Open', key='-COMSTATE-'), LEDIndicator('-COMLED-')],
            [sg.Text('COM Port'), sg.Push(), sg.Combo(['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1'], '/dev/ttyUSB0', key='-PORT-', size=(10,1))],
            [sg.Text('COM Speed'), sg.Push(), sg.Combo(['115200', '230400', '460800', '921600'], '115200', key='-BAUD-', size=(10,1))],
            [sg.Button('Connect', size = 10, key='-CONNECT-')]]

comms_frame = sg.Frame('Communication Settings', comms_col, expand_x=True, pad=(0,5))

camera_col = [[sg.Text('Sensor Status:'), sg.Push(), sg.Text('OFF', key='-CAMSTATE-'), LEDIndicator('-CAMLED-')],
              [sg.Button('Set Mode', size = 10),  sg.Push(), sg.Combo(['OFF', 'SD', 'ZOOM', 'HD'], 'OFF', size = 10, readonly = True, key='-MODE-')],
              [sg.Button('Set Exposure', size = 10), sg.Push(), sg.Text("672", key='-EXP-'), sg.Slider(range=(1,999), size=(16, 19), orientation='h', key='-SLIDE-', expand_x=True, default_value=672, disable_number_display=True, enable_events=True), sg.Sizer(h_pixels = 1, v_pixels = 19)]]

camera_frame = sg.Frame('Image Sensor Settings', camera_col, expand_x=True, pad=(0,5))

capture_col =   [[sg.Text('Memory Slot'), sg.Push(), sg.Combo([0, 1, 2, 3], 0, size = 10, readonly = True, key='-MEMSLOT-')],
                 [sg.Button('Capture', size = 10), sg.Checkbox('Test Pattern', key='-TEST_PATTERN-')],
                 [sg.Button('Download', size = 10), sg.Checkbox('Preview', key='-PREVIEW_DOWNLOAD-'), sg.Push(), sg.Text('Idle', key='-DOWNSTAT-')]]

capture_frame = sg.Frame('Capture and Download', capture_col, expand_x=True, pad=(0,5))

advanced_col =  [[sg.Button('Set Baud', size = 10), sg.Push(), sg.Combo(['115200', '230400', '460800', '921600'], '115200', size = 10, readonly = True, key='-CAM_BAUD-')],
                 [sg.Button('Ping', size = 10), sg.Checkbox('Including CAM', key='-PINGINCL-'), sg.Push(), sg.Text('Time', key='-TIME-')],
                 [sg.Text('Write to Register (Hex values)')],
                 [sg.Button('R', size = 4), sg.Button('W', size=4), sg.Push(), sg.Text("ADDR:"), sg.In(default_text='3000', size=4, key='-ADDR-'), sg.Text('DATA:'), sg.In(default_text='0000', size=4, key='-DAT-')],
                 [sg.Multiline(default_text = "", key='-OUT-', enter_submits=False, autoscroll=True, size=(40,6), expand_x=True, expand_y = True)]]

advanced_frame = sg.Frame('Advanced Settings', advanced_col, expand_x = True, expand_y = True, pad = (0,5))

open_col = [[sg.Text('File'), sg.In(size=(25,1), enable_events=True ,key='-FILE-', expand_x=True), sg.FileBrowse()]]
open_frame = sg.Frame('Open Image from File', open_col, expand_x=True, pad=(0,5))

image_editing_col = [[sg.Button('Debayer', size = 10, key='-DEBAYER-'), sg.Button('Auto Adjust', size = 10, key='-AUTO_ADJ-'), sg.Text('Fix colour and contrast') ]]

image_editing_frame = sg.Frame('Image Editing', image_editing_col, expand_x = True, pad = (0,21))

# Add for raw img
raw_img_col = [[
    sg.Button('Open Raw Image', key='-OPEN_RAW-'),
    sg.Text('', key='-RAW_PATH-', size=(40, 1))
]]
raw_img_frame = sg.Frame('Raw Image Open', raw_img_col, expand_x=True, pad=(0,5))

left_col = [[comms_frame], [camera_frame], [capture_frame], [advanced_frame]]
            

# For now will only show the name of the file that was chosen
images_col = [[open_frame],
              [image_editing_frame],
              [raw_img_frame],
              [sg.Image(key='-IMAGE-')],
              [sg.Text('Open an image using the browser above, or capture a new image using left panel controls.', key='-TOUT-', size=(70,1))]]


# ----- Full layout -----
layout = [[sg.Column(left_col, element_justification='l', expand_y=True), sg.VSeperator(), sg.Column(images_col, element_justification='c', expand_y=True)]]

# --------------------------------- Create Window ---------------------------------
window = sg.Window('MVP Aerospace Camera GUI V2.0', layout,resizable=True, finalize=True)

SetLED(window, '-COMLED-', 'red')
SetLED(window, '-CAMLED-', 'red')

s = serial.Serial()

# ----- Run the Event Loop -----
# --------------------------------- Event Loop ---------------------------------
while True:

    event, values = window.read()  
    
    if event in (sg.WIN_CLOSED, 'Exit'):
        break
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    
    if event == '-CONNECT-':
        try:
            if s.isOpen() == False:
                s.port = window['-PORT-'].get()
                s.baudrate = int(window['-BAUD-'].get())
                s.timeout = 2
                s.open()
                window['-COMSTATE-'].update('CONNECTED')
                window['-CONNECT-'].update('Disconnect')
                append_output_window('COM Port opened.')
            else:
                s.close()
                window['-COMSTATE-'].update('CLOSED')
                window['-CONNECT-'].update('Connect')
                append_output_window('COM Port closed.')
              
        except Exception as E:
            window['-COMSTATE-'].update('Error')
            SetLED(window, '-COMLED-', 'Yellow')
            append_output_window('Comm port error!')
            print(f'** Error {E} **')
            pass
        
    if event == 'Ping':
        if s.isOpen():
            try:

                s.reset_input_buffer()
                if(window['-PINGINCL-'].get() == True):
                    s.write(bytes([0x40, 0x50, 0x01, 0x00, 0x00, 0x00, 0x0D]))
                    append_output_window("Pinging, including camera sensor...")
                else:
                    s.write(bytes([0x40, 0x50, 0x00, 0x00, 0x00, 0x00, 0x0D]))
                    append_output_window("Pinging MCU only...")
                
                result = wait_for_telem()
                if(result):
                    window['-TIME-'].update(str(int((result[2]<<8) + result[3])))
                else:
                    append_output_window("Command timed out.")
                
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass

    if event == 'Set Baud':
        if s.isOpen():
            try:

                s.reset_input_buffer()
                if values['-CAM_BAUD-'] in baudrates:
                    append_output_window("Change KissCAM baud rate to " + values['-CAM_BAUD-'])
                    cam_baud = baudrates.index(values['-CAM_BAUD-'])
                    s.write(bytes([0x40, 0x42, cam_baud, 0x00, 0x00, 0x00, 0x0D]))
                    wait_for_telem() #not expecting any return telemetry other than ack
                    
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass

    if event == 'Set Mode':
        if s.isOpen():
            try:

                s.reset_input_buffer()
                if values['-MODE-'] in mode_names:
                  append_output_window("Setting image sensor mode to " + values['-MODE-'])
                  mode = mode_names.index(values['-MODE-'])
                else:
                  append_output_window("Uknown Mode Selected. Setting to OFF.")
                  mode = 0

                s.write(bytes([0x40, 0x4D, mode, 0x00, 0x00, 0x00, 0x0D]))
                wait_for_telem() #not expecting any return telemetry other than ack
                
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass
          
    if event == '-SLIDE-':
        window['-EXP-'].update("{:03d}".format(int(values['-SLIDE-'])))
        window.refresh

    if event == 'Set Exposure':
        if s.isOpen():
            try:

                s.reset_input_buffer()
                exp = int(values['-SLIDE-'])
                append_output_window("Setting exposure...")

                s.write(bytes([0x40, 0x45, (exp>>8)&0xFF, exp&0xFF, 0x00, 0x00, 0x0D]))
                wait_for_telem() #not expecting any return telemetry other than ack
                
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass
        
    if event == 'Capture':
        if s.isOpen():
            try:

                s.reset_input_buffer()
                mem_slot = values['-MEMSLOT-']
                if(window['-TEST_PATTERN-'].get() == True):
                    s.write(bytes([0x40, 0x43, mem_slot, 0x01, 0x00, 0x00, 0x0D]))
                    print("Capture test pattern to MRAM slot: " + str(mem_slot))
                else:
                    s.write(bytes([0x40, 0x43, mem_slot, 0x00, 0x00, 0x00, 0x0D]))
                    print("Capture to MRAM slot: " + str(mem_slot))
                    
                append_output_window("Capturing...")
                window['-DOWNSTAT-'].update("Captured")
                wait_for_telem() #not expecting any return telemetry other than ack
                
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass

    if event == 'Download':
        if s.isOpen():
            try:

                if(window['-PREVIEW_DOWNLOAD-'].get() == False):
                    x_size = 640
                    y_size = 480
                    preview = 0
                else:
                    x_size = 64
                    y_size = 48
                    preview = 1

                img = PIL.Image.new('RGB', (x_size, y_size), color = 'black')
                pixels = img.load()
                
                y=0
                while y < y_size:    # for every row:
                    s.reset_input_buffer()
                    mem_slot = values['-MEMSLOT-']
                    cmd = [0x40, 0x44, mem_slot, preview, (y>>8)&0xFF, y&0xFF, 0x0D]
                    #cmd = [0x40, 0x44, mem_slot, preview, 0, 0, 0x0D]
                    s.write(bytes(cmd))
                    window['-DOWNSTAT-'].update("Downloading: "  + str(int((y/y_size)*100)) + "%")
                    window.refresh()

                    result = wait_for_telem() # `result` is `LN1 + LN2 + IMG`
                    if len(result) == (x_size+2): # 642
                        for x in range(2,x_size+2):         # For every column (first 2 bytes are line number)
                            dat = result[x]
                            pixels[x-2,y] = (dat,dat,dat)   # set the colour accordingly
                        y = y+1                             #end of line character
                    else:
                        append_output_window("Invalid number of bytes received. Retrying line.")
                        window.refresh()

                window['-DOWNSTAT-'].update("Download Done")
                window.refresh        

                now = datetime.now() # current date and time
                new_filename = "output"+now.strftime("%d%m%Y_%H%M%S")+".bmp"
                img.save(new_filename)

                window['-TOUT-'].update(new_filename)
                window['-IMAGE-'].update(data=convert_to_bytes(new_filename))

            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass

    if event == 'W':
        if s.isOpen():
            try:
                s.reset_input_buffer()
                register_addr = int(window['-ADDR-'].get(), 16)
                register_data = int(window['-DAT-'].get(), 16)
                s.write(bytes([0x40, 0x57, (register_addr>>8) & 0xFF, register_addr&0xFF, (register_data>>8) & 0xFF, register_data&0xFF, 0x0D]))
                append_output_window("Writing to register")
                window.refresh
                result = wait_for_telem() #not expecting any return telemetry other than ack
                
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass

    if event == 'R':
        if s.isOpen():
            try:
                s.reset_input_buffer()
                #register_addr = int(window['-ADDR-'].get())
                register_addr = int(window['-ADDR-'].get(), 16)
                print(register_addr)
                register_data = 0
                s.write(bytes([0x40, 0x52, (register_addr>>8) & 0xFF, register_addr&0xFF, 0x00, 0x00, 0x0D]))
                append_output_window("Reading from register")
                window.refresh
                result = wait_for_telem()
                print("got response")
                if(result):
                    append_output_window("Read Completed")
                    print(result)
                    window['-ADDR-'].update("{:04X}".format((result[0]<<8) + result[1]))
                    window['-DAT-'].update("{:04X}".format((result[2]<<8) + result[3]))
                    window.refresh
                else:
                    append_output_window("Read timed out.")
                    window.refresh
                
            except Exception as E:
                append_output_window(f'** Error {E} **')
                pass
        else:
            append_output_window('Error: COM Port is not open')
            pass
    
        
    if event == '-FILE-':                       # File name chosen. Open image
        filename = values['-FILE-']
        try:
            window['-TOUT-'].update(filename)
            window['-IMAGE-'].update(data=convert_to_bytes(filename))
        except Exception as E:
            print(f'** Error {E} **')
            pass                                # Error opening image file

    if event == '-DEBAYER-':
        filename = window['-TOUT-'].get()
        print('Debayering: ' + filename)
        try:
            im = cv2.imread(filename, 0) #image must be 24bit. 8bit image causes error message
            image = cv2.cvtColor(im, cv2.COLOR_BayerGR2RGB)
            filename = filename.split('.')
            colour_filename = filename[0] + '_colour.bmp'
            print('Saving colour image: ' + colour_filename)
            cv2.imwrite(colour_filename, image)
            window['-TOUT-'].update(colour_filename)
            window['-IMAGE-'].update(data=convert_to_bytes(colour_filename))
        except Exception as E:
            print(f'** Error {E} **')
            pass

    if event == '-AUTO_ADJ-':
        filename = window['-TOUT-'].get()
        print('Auto adjusting: ' + filename)
        try:
            img = cv2.imread(filename, 1) #image must be 24bit. 8bit image causes error message

            # Apply the processing workflow
            img = scale_chn(img, 'r', 5)
            img = scale_chn(img, 'g', 3)
            img = scale_chn(img, 'b', 1)
            img = simplest_cb(img,0.25)
            #img = white_balance(img)
            img = apply_clahe(img)
            #img = adjust_gamma(img,1.2)

            filename = filename.split('.')
            adjusted_filename = filename[0] + '_adjusted.bmp'
            print('Saving adjusted colour image: ' + adjusted_filename)
            cv2.imwrite(adjusted_filename, img)
            window['-TOUT-'].update(adjusted_filename)
            window['-IMAGE-'].update(data=convert_to_bytes(adjusted_filename))

        except Exception as E:
            print(f'** Error {E} **')
            pass
    # Add for raw img dat
    if event == '-OPEN_RAW-':
        path = sg.popup_get_file('Select raw file', no_window=True)
        if path:
            img = parse_raw_file_to_image(path, rgb=True)
            now = datetime.now() # current date and time
            out = "output"+now.strftime("%d%m%Y_%H%M%S")+".bmp"
            img.save(out)
            window['-IMAGE-'].update(data=convert_to_bytes(out))
            window['-TOUT-'].update(out)


    if(window['-CAMSTATE-'].get() == 'OFF'):
      SetLED(window, '-CAMLED-', 'red')
    else:
      SetLED(window, '-CAMLED-', 'green')

    if(window['-COMSTATE-'].get() == 'CONNECTED'):
      SetLED(window, '-COMLED-', 'green')
    else:
      SetLED(window, '-COMLED-', 'red')

        
# --------------------------------- Close & Exit ---------------------------------
window.close()







