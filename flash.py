#!/usr/bin/env python3

import os
import subprocess

def upload_framware(device_path):
	print("Start upload firmware")
	script_path = './Tools/px_uploader.py'

	all_script_arguments = [script_path, '--port', device_path, './build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4']
	command = ["python3"] + all_script_arguments

	try:
		subprocess.run(command, check=True)
	except subprocess.CalledProcessError as e:
		print(f"Error: {e}")

def check_dev(device_path):

        if os.path.exists(device_path):
            return True

        return False


device_pref = "/dev/ttyACM"
for num in "0", "1":
	dev_name = device_pref + num
	if (check_dev(dev_name)):
		print("Device ", dev_name, " exists")
		upload_framware(dev_name)
		break


