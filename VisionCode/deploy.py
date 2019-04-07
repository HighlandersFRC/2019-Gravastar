import os
import tkinter as tk
import sys
import glob
import serial
import time
import shutil

class OptionMenu(tk.OptionMenu):
	def __init__(self, *args, **kw):
		self._command = kw.get("command")
		self.variable = args
		tk.OptionMenu.__init__(self, *self.variable, **kw)
		
	def addOption(self, label):
		self["menu"].add_command(label=label,
			command=tk._setit(self.variable[1], label, self._command))
	def deleteAll(self):
		self["menu"].delete(0, "end")

def deploy(port, module):
	try:
		com = serial.Serial(port)
		com.write(b'streamoff\r')
		com.write(b'usbsd\r')
		time.sleep(3)
		srcFile = "./modules/Highlanders/" + module + "/" + module + ".py"
		dstDir = "D:/modules/Highlanders/" + module
			
		shutil.copy2(srcFile, dstDir)
		com.write(b'restart\r')
		com.close()
	except (serial.SerialException) as e:
		print("Could not connect to", port, e)
	except (OSError) as e:
		print(e)


def refresh(commMenu, moduleMenu):
	refreshModules(moduleMenu)
	refreshPorts(commMenu)
		
def refreshModules(menu):
	try:
		modules = os.listdir("./modules/Highlanders")
	except OSError as e:
		print("No Modules Found", e)	
	if (len(modules) == 0):
		modules = ['']	
	
	menu.deleteAll()
	for mod in modules:
		menu.addOption(mod)
		
def refreshPorts(menu):
	comms = serial_ports()

	if (len(comms) == 0):
		comms = ['']

	menu.deleteAll()
	for comm in comms:
		menu.addOption(comm)
	
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
	main = tk.Tk()
	
	moduleString = tk.StringVar(main)
	moduleString.set([''])
	moduleMenu = OptionMenu(main, moduleString, "")
	moduleMenu.pack()
	
	commString = tk.StringVar(main)
	commString.set([''])
	commMenu = OptionMenu(main, commString, "")
	commMenu.pack()
	
	refresh(commMenu, moduleMenu)
	
	button = tk.Button(main, text='Cancel', width=25, command=main.destroy)
	button.pack()

	button = tk.Button(main, text='Deploy', width=25, command=lambda : deploy(commString.get(), moduleString.get()))
	button.pack()
	
	button = tk.Button(main, text='Refresh', width=25, command=lambda : refresh(commMenu, moduleMenu))
	button.pack()
	
	main.mainloop()
