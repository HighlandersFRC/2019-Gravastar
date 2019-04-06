import os
import tkinter as tk
#import pxssh

import sys
import glob
import serial

def deploy(port, module):
	try:
		com = serial.Serial(port, 115200)
		com.write("streamoff\n")
		com.write("usbsd\n")
		com.close()
	except (OSError, serial.SerialException):
		print("Could not connect to %s", port)
	

def serial_ports():
	""" Lists serial port names

		:raises EnvironmentError:
			On unsupported or unknown platforms
		:returns:
			A list of the serial ports available on the system
	"""
	if sys.platform.startswith('win'):
		ports = ['COM%s' % (i + 1) for i in range(256)]
	elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
		# this excludes your current terminal "/dev/tty"
		ports = glob.glob('/dev/tty[A-Za-z]*')
	elif sys.platform.startswith('darwin'):
		ports = glob.glob('/dev/tty.*')
	else:
		raise EnvironmentError('Unsupported platform')

	result = []
	for port in ports:
		try:
			s = serial.Serial(port)
			s.close()
			result.append(port)
		except (OSError, serial.SerialException):
			pass
	return result


if __name__ == '__main__':

	try:
		modules = os.listdir("./modules/Highlanders")
	except:
		print("No Modules Found")
		
	coms = serial_ports()
	
	if (len(modules) == 0):
		modules = ['']
	if (len(coms) == 0):
		coms = ['']
		
	main = tk.Tk()

	moduleString = tk.StringVar(main)
	moduleString.set(modules[0])
	w = tk.OptionMenu(main, moduleString, *modules)
	w.pack()

	comString = tk.StringVar(main)
	comString.set(coms[0])
	w = tk.OptionMenu(main, comString, *coms)
	w.pack()
	
	button = tk.Button(main, text='Cancel', width=25, command=main.destroy)
	button.pack()

	button = tk.Button(main, text='Deploy', width=25, command=deploy)
	button.pack()
	
	main.mainloop()
