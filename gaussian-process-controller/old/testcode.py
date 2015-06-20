print 'ciao'
fd = open("out.txt","r")
line = fd.readline().split()
value = int(line[0])
print value,value+1
fd.close()
fd = open("out.txt","w")
fd.write(str(value+10))
fd.close()
fd = open("out.txt","r")
line = fd.readline().split()
value = int(line[0])
print value,value+1

