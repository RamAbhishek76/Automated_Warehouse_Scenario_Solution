import os
import subprocess

valid = False
time = 10
instance_code = 1
 
while not valid:
	print()
	print('code'.ljust(5), 'Corresponding File'.ljust(8))
	for i in range(1,6):
		print(str(i).ljust(5),'inst'+str(i)+'.asp'.ljust(8))
	print()

	instance_code = input('Please enter the code for the instance file: ')
	if (instance_code.isdigit()) and (int(instance_code) in range(1,6)):
		valid = True
	else:
		print()
		print("-"*10, "ERROR", "-"*10)
		print("Please Enter a valid code")

print("\n\n")
print("-"*10, "EXECUTING", "-"*10)
print("\n\n")

while valid:
	out = subprocess.Popen(['clingo', 'automatedwarehousesystem.asp', os.path.join('simpleInstances','inst'+str(instance_code)+'.asp'), '-c', 'n='+str(time)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
	stdout,stderr = out.communicate()
	
	if 'UNSATISFIABLE' in stdout.decode("utf-8").split():
		time = time + 2
	else:
		valid = False
		print(stdout.decode("utf-8"))