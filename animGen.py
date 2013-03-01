input = open("anim_raw", 'r')
output = open("anim_turn", 'w')

frames = {}

for line in input:
	tokens = line.strip().split(" ")
	time = float(tokens[0])
	tokens = tokens[1:]
	fl = []
	for t in tokens:
		fl.append(float(t))
	fours = zip(fl[0::4], fl[1::4], fl[2::4], fl[3::4])
	frames[time] = {}
	for x in fours:
		frames[time][x[0]] = x
times = frames.keys()
times.append(0)
times.sort()
timepairs = zip(times,times[1:])
for tp in timepairs:
	for bone in frames[tp[1]]:
		if(int(float(tp[0]))==0):
			output.write("{} {} {} 0 0 0 {} {} {}\n".format(tp[0], tp[1],int(bone), frames[tp[1]][bone][1],frames[tp[1]][bone][2],frames[tp[1]][bone][3]  ))
		else:
			output.write("\n{} {} {} {} {} {} {} {} {}".format(tp[0], tp[1],int(bone), frames[tp[0]][bone][1],frames[tp[0]][bone][2],frames[tp[0]][bone][3],frames[tp[1]][bone][1],frames[tp[1]][bone][2],frames[tp[1]][bone][3]  ))

output.close()



# endtime b x y z b x y z b x y z