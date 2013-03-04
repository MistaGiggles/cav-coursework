attach=open("attachment2.out",'r')
attachments = []
for x in attach:
	attachments.append(x.strip().split(" "))
attach.close()
skeleton = []
skele=open("skeleton2.out",'r')
for x in skele:
	skeleton.append(x.strip().split(" "))
skele.close()
vertices = []
trianges = []
vert=open("arma2.obj")
for line in vert:
		tokens = line.strip().split(" ")
		if(tokens[0]=='v'):
				vertices.append(tokens)
vert.close()